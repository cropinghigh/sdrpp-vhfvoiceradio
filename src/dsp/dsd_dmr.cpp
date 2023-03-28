#include "dsd.h"

namespace dsp { 
    void DSD::processDMRdata () {
        int i, dibit;
        int *dibit_p;
        char sync[25];
        char syncdata[25];
        char cachdata[13];
        char cc[5];
        char bursttype[5];

        cc[4] = 0;
        bursttype[4] = 0;

        dibit_p = dibitBufP - 90;

        // CACH
        for (i = 0; i < 12; i++) {
            dibit = *dibit_p;
            dibit_p++;
            if (invertedDmr == 1) {
                dibit = (dibit ^ 2);
            }
            cachdata[i] = dibit;
            if (i == 2) {
                currentslot = (1 & (dibit >> 1));      // bit 1
                if (currentslot == 0) {
                    slot0light[0] = '[';
                    slot0light[6] = ']';
                    slot1light[0] = ' ';
                    slot1light[6] = ' ';
                } else {
                    slot1light[0] = '[';
                    slot1light[6] = ']';
                    slot0light[0] = ' ';
                    slot0light[6] = ' ';
                }
            }
        }
        cachdata[12] = 0;

        // current slot
        dibit_p += 49;

        // slot type
        dibit = *dibit_p;
        dibit_p++;
        if (invertedDmr == 1) {
            dibit = (dibit ^ 2);
        }
        cc[0] = (1 & (dibit >> 1)) + 48;      // bit 1
        cc[1] = (1 & dibit) + 48;     // bit 0

        dibit = *dibit_p;
        dibit_p++;
        if (invertedDmr == 1) {
            dibit = (dibit ^ 2);
        }
        cc[2] = (1 & (dibit >> 1)) + 48;      // bit 1
        cc[3] = (1 & dibit) + 48;     // bit 0

        dibit = *dibit_p;
        dibit_p++;
        if (invertedDmr == 1) {
            dibit = (dibit ^ 2);
        }
        bursttype[0] = (1 & (dibit >> 1)) + 48;       // bit 1
        bursttype[1] = (1 & dibit) + 48;      // bit 0

        dibit = *dibit_p;
        dibit_p++;
        if (invertedDmr == 1) {
            dibit = (dibit ^ 2);
        }
        bursttype[2] = (1 & (dibit >> 1)) + 48;       // bit 1
        bursttype[3] = (1 & dibit) + 48;      // bit 0

        // parity bit
        dibit = *dibit_p;
        dibit_p++;

        if (strcmp (bursttype, "0000") == 0) {
            sprintf(fsubtype, " PI Header    ");
        } else if (strcmp (bursttype, "0001") == 0) {
            sprintf(fsubtype, " VOICE Header ");
        } else if (strcmp (bursttype, "0010") == 0) {
            sprintf(fsubtype, " TLC          ");
        } else if (strcmp (bursttype, "0011") == 0) {
            sprintf(fsubtype, " CSBK         ");
        } else if (strcmp (bursttype, "0100") == 0) {
            sprintf(fsubtype, " MBC Header   ");
        } else if (strcmp (bursttype, "0101") == 0) {
            sprintf(fsubtype, " MBC          ");
        } else if (strcmp (bursttype, "0110") == 0) {
            sprintf(fsubtype, " DATA Header  ");
        } else if (strcmp (bursttype, "0111") == 0) {
            sprintf(fsubtype, " RATE 1/2 DATA");
        } else if (strcmp (bursttype, "1000") == 0) {
            sprintf(fsubtype, " RATE 3/4 DATA");
        } else if (strcmp (bursttype, "1001") == 0) {
            sprintf(fsubtype, " Slot idle    ");
        } else if (strcmp (bursttype, "1010") == 0) {
            sprintf(fsubtype, " Rate 1 DATA  ");
        } else {
            sprintf(fsubtype, "              ");
        }

        // signaling data or sync
        for (i = 0; i < 24; i++) {
            dibit = *dibit_p;
            dibit_p++;
            if (invertedDmr == 1) {
                dibit = (dibit ^ 2);
            }
            syncdata[i] = dibit;
            sync[i] = (dibit | 1) + 48;
        }
        sync[24] = 0;
        syncdata[24] = 0;

        if ((strcmp (sync, DMR_BS_DATA_SYNC) == 0) || (strcmp (sync, DMR_MS_DATA_SYNC) == 0)) {
            if (currentslot == 0) {
                sprintf(slot0light, "[slot0]");
            } else {
                sprintf(slot1light, "[slot1]");
            }
        }

        if (errorbars == 1) {
//            printf ("%s %s ", slot0light, slot1light);
            if(currentslot == 0) {
                status_last_dmr_slot0_burst = fsubtype;
            } else {
                status_last_dmr_slot1_burst = fsubtype;
            }

        }

        // current slot second half, cach, next slot 1st half
        skipDibit (120);

        if (errorbars == 1) {
            if (strcmp (fsubtype, "              ") == 0) {
//                printf (" Unknown burst type: %s\n", bursttype);
            } else {
//                printf ("%s\n", fsubtype);
            }
        }
    }

    void DSD::processDMRvoice () {
        // extracts AMBE frames from DMR frame
        int i, j, dibit;
        int *dibit_p;
        char ambe_fr[4][24];
        char ambe_fr2[4][24];
        char ambe_fr3[4][24];
        const int *w, *x, *y, *z;
        char sync[25];
        char syncdata[25];
        char cachdata[13];
        int mutecurrentslot;
        int msMode;

        mutecurrentslot = 0;
        msMode = 0;

        dibit_p = dibitBufP - 144;
        for (j = 0; j < 6; j++) {
            // 2nd half of previous slot
            for (i = 0; i < 54; i++) {
                if (j > 0) {
                    dibit = getDibit ();
                } else {
                    dibit = *dibit_p;
                    dibit_p++;
                    if (invertedDmr == 1) {
                        dibit = (dibit ^ 2);
                    }
                }
            }

            // CACH
            for (i = 0; i < 12; i++) {
                if (j > 0) {
                    dibit = getDibit ();
                } else {
                    dibit = *dibit_p;
                    dibit_p++;
                    if (invertedDmr == 1) {
                        dibit = (dibit ^ 2);
                    }
                }
                cachdata[i] = dibit;
                if (i == 2) {
                    currentslot = (1 & (dibit >> 1));  // bit 1
                    if (currentslot == 0) {
                        slot0light[0] = '[';
                        slot0light[6] = ']';
                        slot1light[0] = ' ';
                        slot1light[6] = ' ';
                    } else {
                        slot1light[0] = '[';
                        slot1light[6] = ']';
                        slot0light[0] = ' ';
                        slot0light[6] = ' ';
                    }
                }
            }
            cachdata[12] = 0;

            // current slot frame 1
            w = rW;
            x = rX;
            y = rY;
            z = rZ;
            for (i = 0; i < 36; i++) {
                if (j > 0) {
                    dibit = getDibit ();
                } else {
                    dibit = *dibit_p;
                    dibit_p++;
                    if (invertedDmr == 1) {
                        dibit = (dibit ^ 2);
                    }
                }
                ambe_fr[*w][*x] = (1 & (dibit >> 1)); // bit 1
                ambe_fr[*y][*z] = (1 & dibit);        // bit 0
                w++;
                x++;
                y++;
                z++;
            }

            // current slot frame 2 first half
            w = rW;
            x = rX;
            y = rY;
            z = rZ;
            for (i = 0; i < 18; i++) {
                if (j > 0) {
                    dibit = getDibit ();
                } else {
                    dibit = *dibit_p;
                    dibit_p++;
                    if (invertedDmr == 1) {
                        dibit = (dibit ^ 2);
                    }
                }
                ambe_fr2[*w][*x] = (1 & (dibit >> 1));        // bit 1
                ambe_fr2[*y][*z] = (1 & dibit);       // bit 0
                w++;
                x++;
                y++;
                z++;
            }

            // signaling data or sync
            for (i = 0; i < 24; i++) {
                if (j > 0) {
                    dibit = getDibit ();
                } else {
                    dibit = *dibit_p;
                    dibit_p++;
                    if (invertedDmr == 1) {
                        dibit = (dibit ^ 2);
                    }
                }
                syncdata[i] = dibit;
                sync[i] = (dibit | 1) + 48;
            }
            sync[24] = 0;
            syncdata[24] = 0;

            if ((strcmp (sync, DMR_BS_DATA_SYNC) == 0) || (strcmp (sync, DMR_MS_DATA_SYNC) == 0)) {
                mutecurrentslot = 1;
                if (currentslot == 0) {
                    sprintf(slot0light, "[slot0]");
                } else {
                    sprintf(slot1light, "[slot1]");
                }
            } else if ((strcmp (sync, DMR_BS_VOICE_SYNC) == 0) || (strcmp (sync, DMR_MS_VOICE_SYNC) == 0)) {
                mutecurrentslot = 0;
                if (currentslot == 0) {
                    sprintf(slot0light, "[SLOT0]");
                } else {
                    sprintf(slot1light, "[SLOT1]");
                }
            }
            if ((strcmp (sync, DMR_MS_VOICE_SYNC) == 0) || (strcmp (sync, DMR_MS_DATA_SYNC) == 0)) {
                msMode = 1;
            }

            if ((j == 0) && (errorbars == 1)) {
//                printf ("%s %s  VOICE e:", slot0light, slot1light);
                status_mbedecoding = true;
                status_errorbar = "";
                if(currentslot == 0) {
                    status_last_dmr_slot0_burst = "VOICE";
                } else {
                    status_last_dmr_slot1_burst = "VOICE";
                }
            }

            // current slot frame 2 second half
            for (i = 0; i < 18; i++) {
                dibit = getDibit ();
                ambe_fr2[*w][*x] = (1 & (dibit >> 1));        // bit 1
                ambe_fr2[*y][*z] = (1 & dibit);       // bit 0
                w++;
                x++;
                y++;
                z++;
            }

            if (mutecurrentslot == 0) {
                if (firstframe == 1) {
                    // we don't know if anything received before the first sync after no carrier is valid
                    firstframe = 0;
                } else {
                    processMbeFrame (NULL, ambe_fr, NULL);
                    processMbeFrame (NULL, ambe_fr2, NULL);
                }
            }

            // current slot frame 3
            w = rW;
            x = rX;
            y = rY;
            z = rZ;
            for (i = 0; i < 36; i++) {
                dibit = getDibit ();
                ambe_fr3[*w][*x] = (1 & (dibit >> 1));        // bit 1
                ambe_fr3[*y][*z] = (1 & dibit);       // bit 0
                w++;
                x++;
                y++;
                z++;
            }
            if (mutecurrentslot == 0) {
                processMbeFrame (NULL, ambe_fr3, NULL);
            }

            // CACH
            for (i = 0; i < 12; i++) {
                dibit = getDibit ();
                cachdata[i] = dibit;
            }
            cachdata[12] = 0;

            // next slot
            skipDibit (54);

            // signaling data or sync
            for (i = 0; i < 24; i++) {
                dibit = getDibit ();
                syncdata[i] = dibit;
                sync[i] = (dibit | 1) + 48;
            }
            sync[24] = 0;
            syncdata[24] = 0;

            if ((strcmp (sync, DMR_BS_DATA_SYNC) == 0) || (msMode == 1)) {
                if (currentslot == 0) {
                    sprintf(slot1light, " slot1 ");
                } else {
                    sprintf(slot0light, " slot0 ");
                }
            } else if (strcmp (sync, DMR_BS_VOICE_SYNC) == 0) {
                if (currentslot == 0) {
                    sprintf(slot1light, " SLOT1 ");
                } else {
                    sprintf(slot0light, " SLOT0 ");
                }
            }

            if (j == 5) {
                // 2nd half next slot
                skipDibit (54);

                // CACH
                skipDibit (12);

                // first half current slot
                skipDibit (54);
            }
        }

        if (errorbars == 1) {
            status_mbedecoding = false;
//            printf ("\n");
        }
    }
};
