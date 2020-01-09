#define ZERYNTH_PRINTF
#include "zerynth.h"

#define NMEA_DEBUG 0

#if NMEA_DEBUG
#define DEBUG(...) printf(__VA_ARGS__)
#else
#define DEBUG(...)
#endif

int vatoi(uint8_t *s, int32_t len, uint32_t base, err_t *err);
double vatof(uint8_t *s, int32_t len, err_t *err);

C_NATIVE(_nmea_readline){
    NATIVE_UNWARN();
    int32_t ser;
    uint8_t *buf;
    int32_t buflen;
    int32_t timeout=5000;

    if (parse_py_args("isi", nargs, args, &ser, &buf, &buflen, &timeout) != 3) {
        *res = MAKE_NONE();
        return ERR_TYPE_EXC;
    }
    ser&=0xFF;

    int bytes=0,dollar=0;
    uint8_t *ptr = buf;
    uint8_t *end = NULL;
    uint32_t crc=0,lcrc=0;

    RELEASE_GIL();
    uint32_t tstart = vosMillis();
    while (bytes < buflen-1) {
        if (timeout > 0) {
            if ((vosMillis() - tstart) > timeout) {
                buf++;
                *buf=0;
                ACQUIRE_GIL();
                *res = MAKE_NONE();
                return ERR_TIMEOUT_EXC;
            }
            if (vhalSerialAvailable(ser) > 0) {
                vhalSerialRead(ser, ptr, 1);
                tstart = vosMillis();
            } else {
                vosThSleep(TIME_U(50, MILLIS));
                continue;
            }
        } else {
            vhalSerialRead(ser, ptr, 1);
        }
        bytes++;
        if (*ptr == '$'){
            // restart
            ptr = buf;
            *ptr = '$';
            bytes = 1;
            dollar = 1;
        }
        else if (*ptr == '\r' || *ptr == '\n') {
            if (dollar)
                break;
            // restart
            ptr = buf;
        }
        ptr++;
    }
    ACQUIRE_GIL();
    *ptr = 0;

    // checks
    if (*buf != '$'){
        //invalid line
        *res = PSMALLINT_NEW(-1);
    }
    else {
        for(ptr=buf+1; ptr<buf+bytes; ++ptr) {
            if (*ptr=='*') {
                end=ptr;
                break;
            }
            crc = crc^*ptr;
        }
    }
    if (!end){
        //invalid line
        *res = PSMALLINT_NEW(-2);
    }
    else {
        //read checksum
        uint8_t ch=*(end+1);
        if (ch>='0'&&ch<='9') lcrc += (ch-'0')*16;
        else if(ch>='A'&&ch<='F') lcrc += (ch-'A'+10)*16;
        else lcrc=-1;
        ch=*(end+2);
        if (ch>='0'&&ch<='9') lcrc += (ch-'0');
        else if(ch>='A'&&ch<='F') lcrc += (ch-'A'+10);
        else lcrc=-1;

        if(crc!=lcrc) {
            //bad checksum
            *res = PSMALLINT_NEW(-3);
        } else {
            *res = PSMALLINT_NEW((end-buf));
        }
    }
    
    DEBUG("> %i %s\n",bytes,buf);
    return ERR_OK;
}


typedef struct _splits {
    uint8_t* start;
    int len;
} NmeaSplit;
NmeaSplit nmea_flds[20];

int _do_split(uint8_t *line, int len, NmeaSplit *flds, int fldlen) {
    int i,j=0;
    uint8_t *start = line;
    memset(flds,0,sizeof(NmeaSplit)*fldlen);
    for (i=0;i<len;i++) {
        if (j>=fldlen) break;
        if (line[i]==',') {
            flds[j].start = start;
            flds[j].len = (line+i)-start;
            start = line+i+1;
            j++;
        }
    }
    return j; // num of fields (not counting first cmd)
}

int _nmea_set_time(NmeaSplit *tm, NmeaSplit *dt, int *yy, int *mt, int *dd, int* hh, int* mm, int *ss,int*uu){
    err_t err;
    //parse date: ddmmyy or ddmmyyyy
    *yy = vatoi(dt->start+4,dt->len-3,10,&err);

    if (*yy>=100) {
    } else if(*yy>=80) {
        *yy=1900+*yy;
    }else { 
        *yy=2000+*yy;
    }
    *mt = vatoi(dt->start+2,2,10,&err); 
    *dd = vatoi(dt->start,2,10,&err); 

    //parse time
    *hh = vatoi(tm->start,2,10,&err);
    *mm = vatoi(tm->start+2,2,10,&err);
    *ss = vatoi(tm->start+4,2,10,&err);
    *uu = vatoi(tm->start+7,tm->len-6,10,&err);
    
    return 1;
}


int _nmea_set_pos(NmeaSplit *latf,NmeaSplit *latpf,NmeaSplit *lonf, NmeaSplit *lonpf, double *lat, double *lon){
    err_t err;
	int i;
	for (i=latf->len; i>0; --i)
		if (latf->start[i] == '.') {
			i -= 2;
			break;
		}
    *lat = vatof(latf->start+i,latf->len-i,&err) / 60.0 + vatoi(latf->start,i,10,&err);
	for (i=lonf->len; i>0; --i)
		if (lonf->start[i] == '.') {
			i -= 2;
			break;
		}
    *lon = vatof(lonf->start+i,lonf->len-i,&err) / 60.0 + vatoi(lonf->start,i,10,&err);
    if(*(latpf->start)=='S') *lat = -1.0*(*lat);
    if(*(lonpf->start)=='W') *lon = -1.0*(*lon);
    return 1;
}

int _nmea_set_spd(NmeaSplit *spdf, double *spd){
    err_t err;
    *spd = vatof(spdf->start,spdf->len,&err)*1.852; //knots to km/h
    return 1;
}

int _nmea_set_cog(NmeaSplit *cogf, double *cog){
    err_t err;
    *cog = vatof(cogf->start,cogf->len,&err);
    return 1;
}

C_NATIVE(_nmea_parseline){
    NATIVE_UNWARN();
    uint8_t *buf;
    int32_t buflen;
    int32_t len;

    if (nargs != 4 || parse_py_args("si", 2, args, &buf, &buflen, &len) != 2)
        return ERR_TYPE_EXC;
    if (PTYPE(args[2]) != PLIST || PTYPE(args[3]) != PLIST)
        return ERR_TYPE_EXC;
    PList *tm = (PList *)args[2];
    PList *fix = (PList *)args[3];

    DEBUG("parse buflen=%d len=%d\n",buflen,len);

    uint8_t *end = buf+len;
    uint8_t *start = buf;
    uint8_t *line = start;
    int ln = end-start;
    uint8_t *cmd = line+3;
    int fldn;
    int has_time=0,gcmd=0,has_fix=0;
    int yy,mt,dd,hh,mm,ss,uu,nfix;
    err_t err=0;
    double lat,lon,alt,spd,cog,hdop,vdop,pdop;
    NmeaSplit *flds=nmea_flds;

    if (ln<8) {
        *res = PSMALLINT_NEW(0);
        return ERR_OK;
    }

    //vosThSleep(TIME_U(10,MILLIS));
    RELEASE_GIL();
    
    if(memcmp(cmd,"RMC",3)==0) {
        fldn = _do_split(line,ln,flds,20);
        DEBUG("rmc split %i\n",fldn);
        if (fldn<12) {
            gcmd = 0;
        } else {
            if (*flds[2].start!='A') {
                //no fix 
            } else {
                gcmd = 1;
                has_time = _nmea_set_time(&flds[1],&flds[9],&yy,&mt,&dd,&hh,&mm,&ss,&uu);
                has_fix++;
                _nmea_set_pos(&flds[3],&flds[4],&flds[5],&flds[6],&lat,&lon);
                _nmea_set_spd(&flds[7],&spd);
                _nmea_set_cog(&flds[8],&cog);
            }
        }
    } else if (memcmp(cmd,"GGA",3)==0) {
        fldn = _do_split(line,ln,flds,20);
        DEBUG("gga split %i\n",fldn);
        if (fldn<14) {
            gcmd = 0;
        } else {
            if (*flds[6].start=='0') {
                gcmd=0;
            } else {
                gcmd=2;
                has_fix++;
                hdop = vatof(flds[8].start,flds[8].len,&err);
                alt = vatof(flds[9].start,flds[9].len,&err);
                nfix = vatoi(flds[7].start,flds[7].len,10,&err);
            }
        }
    } else if (memcmp(cmd,"GSA",3)==0) {
        fldn = _do_split(line,ln,flds,20);
        DEBUG("gsa split %i\n",fldn);
        if (fldn<18) {
            gcmd = 0;
        } else {
            if (*flds[2].start!='3') {
                gcmd=0;
            } else {
                gcmd=3;
                has_fix++;
                hdop = vatof(flds[16].start,flds[17].len,&err);
                vdop = vatof(flds[17].start,flds[17].len,&err);
                pdop = vatof(flds[15].start,flds[15].len,&err);
            }
        }
    } else {
        gcmd = 0;
    }

    ACQUIRE_GIL();

    if (has_time){
        PLIST_SET_ITEM(tm,0,PSMALLINT_NEW(yy));
        PLIST_SET_ITEM(tm,1,PSMALLINT_NEW(mt));
        PLIST_SET_ITEM(tm,2,PSMALLINT_NEW(dd));
        PLIST_SET_ITEM(tm,3,PSMALLINT_NEW(hh));
        PLIST_SET_ITEM(tm,4,PSMALLINT_NEW(mm));
        PLIST_SET_ITEM(tm,5,PSMALLINT_NEW(ss));
        PLIST_SET_ITEM(tm,6,PSMALLINT_NEW(uu));
        DEBUG("has_time %i %i %i %i %i %i %i\n",yy,mt,dd,hh,mm,ss,uu);
    }

    if(has_fix){
        if (gcmd==1) {
            PLIST_SET_ITEM(fix,0,pfloat_new(lat));
            PLIST_SET_ITEM(fix,1,pfloat_new(lon));
            PLIST_SET_ITEM(fix,3,pfloat_new(spd));
            PLIST_SET_ITEM(fix,4,pfloat_new(cog));
        } else if(gcmd==2)   {
            PLIST_SET_ITEM(fix,6,pfloat_new(hdop));
            PLIST_SET_ITEM(fix,2,pfloat_new(alt));
            PLIST_SET_ITEM(fix,5,PSMALLINT_NEW(nfix));
        } else if(gcmd==3){
            PLIST_SET_ITEM(fix,6,pfloat_new(hdop));
            PLIST_SET_ITEM(fix,7,pfloat_new(vdop));
            PLIST_SET_ITEM(fix,8,pfloat_new(pdop));
        }
    }

    gcmd=(has_time<<2)+gcmd;
    *res = PSMALLINT_NEW(gcmd);
    return ERR_OK;
}
