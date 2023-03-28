#include "dsd.h"

namespace dsp { 
    void DSD::P25processlcw (char *lcformat, char *mfid, char *lcinfo) {
        char tgid[17], tmpstr[255];
        long talkgroup, source;
        int i, j;

        tgid[16] = 0;

        if (p25lc == 1) {
//            printf ("lcformat: %s mfid: %s lcinfo: %s ", lcformat, mfid, lcinfo);
            if (p25tg == 0) {
//                printf ("\n");
            }
        }

        if (strcmp (lcformat, "00000100") == 0) {
            // first tg is the active channel
            j = 0;
            for (i = 40; i < 52; i++) {
                if (tgcount < 24) {
                    tg[tgcount][j] = lcinfo[i];
                }
                tmpstr[j] = lcinfo[i];
                j++;
            }
            tmpstr[12] = 48;
            tmpstr[13] = 48;
            tmpstr[14] = 48;
            tmpstr[15] = 48;
            tmpstr[16] = 0;
            talkgroup = strtol (tmpstr, NULL, 2);
            lasttg = talkgroup;
            if (tgcount < 24) {
                tgcount = tgcount + 1;
            }
            if (p25tg == 1) {
//                printf ("tg: %li ", talkgroup);
            }

            if (p25tg == 1) {
//                printf ("tg: %li ", talkgroup);

                // the remaining 3 appear to be other active tg's on the system
                j = 0;
                for (i = 28; i < 40; i++) {
                    tmpstr[j] = lcinfo[i];
                    j++;
                }
                tmpstr[12] = 48;
                tmpstr[13] = 48;
                tmpstr[14] = 48;
                tmpstr[15] = 48;
                tmpstr[16] = 0;
                talkgroup = strtol (tmpstr, NULL, 2);
//                printf ("%li ", talkgroup);
                j = 0;
                for (i = 16; i < 28; i++) {
                    tmpstr[j] = lcinfo[i];
                    j++;
                }
                tmpstr[12] = 48;
                tmpstr[13] = 48;
                tmpstr[14] = 48;
                tmpstr[15] = 48;
                tmpstr[16] = 0;
                talkgroup = strtol (tmpstr, NULL, 2);
//                printf ("%li ", talkgroup);
                j = 0;
                for (i = 4; i < 16; i++) {
                    tmpstr[j] = lcinfo[i];
                    j++;
                }
                tmpstr[12] = 48;
                tmpstr[13] = 48;
                tmpstr[14] = 48;
                tmpstr[15] = 48;
                tmpstr[16] = 0;
                talkgroup = strtol (tmpstr, NULL, 2);
//
//                printf ("%li\n", talkgroup);
            }
        } else if (strcmp (lcformat, "00000000") == 0) {
            j = 0;
            if (strcmp (mfid, "10010000") == 0) {
                for (i = 20; i < 32; i++) {
                    if (tgcount < 24) {
                        tg[tgcount][j] = lcinfo[i];
                    }
                    tmpstr[j] = lcinfo[i];
                    j++;
                }
                tmpstr[12] = 48;
                tmpstr[13] = 48;
                tmpstr[14] = 48;
                tmpstr[15] = 48;
            } else {
                for (i = 16; i < 32; i++) {
                    if (tgcount < 24) {
                        tg[tgcount][j] = lcinfo[i];
                    }
                    tmpstr[j] = lcinfo[i];
                    j++;
                }
            }
            tmpstr[16] = 0;
            talkgroup = strtol (tmpstr, NULL, 2);
            lasttg = talkgroup;
            if (tgcount < 24) {
                tgcount = tgcount + 1;
            }
            if (p25tg == 1) {
//                printf ("tg: %li ", talkgroup);
            }

            j = 0;
            for (i = 32; i < 56; i++) {
                tmpstr[j] = lcinfo[i];
                j++;
            }
            tmpstr[24] = 0;
            source = strtol (tmpstr, NULL, 2);
            lastsrc = source;
            if (p25tg == 1) {
//                printf ("src: %li emr: %c\n", source, lcinfo[0]);
            }
        } else if ((p25tg == 1) && (p25lc == 1)) {
//            printf ("\n");
        }
    }

    int DSD::P25checkNID(char* bch_code, int* new_nac, char* new_duid, unsigned char parity) {
        int result;

        // Fill up with the given input
        itpp::bvec input(63);
        for(unsigned int i=0; i<63; i++) {
            input[i] = bch_code[i];
        }

        // Decode it
        itpp::bvec decoded, cw_isvalid;
        bool ok = bch.decode(input, decoded, cw_isvalid);

        if (!ok) {
            // Decode failed
            result = 0;
        } else {
            // Take the NAC from the decoded output. It's a 12 bit number starting from position 0.
            // Just convert the output from a binary sequence to an integer.
            int nac = 0;
            for (int i=0; i<12; i++) {
                nac <<= 1;
                nac |= (int)decoded[i];
            }
            *new_nac = nac;

            // Take the fixed DUID from the encoded output. 4 bit value starting at position 12.
            unsigned char new_duid_0 = (((int)decoded[12])<<1) + ((int)decoded[13]);
            unsigned char new_duid_1 = (((int)decoded[14])<<1) + ((int)decoded[15]);
            new_duid[0] = new_duid_0 + '0';
            new_duid[1] = new_duid_1 + '0';
            new_duid[2] = 0;    // Null terminate

            // Check the parity
            unsigned char expected_parity = parity_table.get_value(new_duid_0, new_duid_1);

            if (expected_parity != parity) {
                // Ignore, not sure what to do
                //printf("Error in parity detected?");
            }

            result = 1;
        }

        return result;
    }

    int DSD::P25readDibit (char* output, int* status_count, int* analog_signal, int* did_read_status) {
        int dibit;
        int status;

        if (*status_count == 35) {

            // Status bits now
            status = getDibit ();
            // TODO: do something useful with the status bits...
            if (did_read_status != NULL) {
                *did_read_status = 1;
            }
            *status_count = 1;

        } else {
            if (did_read_status != NULL) {
                *did_read_status = 0;
            }
            (*status_count)++;
        }

        dibit = getDibitAndAnalogSignal (analog_signal);
        output[0] = (1 & (dibit >> 1));      // bit 1
        output[1] = (1 & dibit);             // bit 0

        return dibit;
    }

    void DSD::P25readDibitUpdateAnalogData (char* output, unsigned int count, int* status_count, P25_Heuristics::P25_AnalogSignal* analog_signal_array, int* analog_signal_index) {
        unsigned int i;
        unsigned int debug_left, debug_right;

        for (i=0; i<count; i+=2) {
            // We read two bits on each call
            int analog_signal;
            int did_read_status;
            int dibit;

            dibit = P25readDibit(output + i, status_count, &analog_signal, &did_read_status);

            if (analog_signal_array != NULL) {
                // Fill up the P25_Heuristics::P25_AnalogSignal struct
                analog_signal_array[*analog_signal_index].value = analog_signal;
                analog_signal_array[*analog_signal_index].dibit = dibit;
                analog_signal_array[*analog_signal_index].sequence_broken = did_read_status;
                (*analog_signal_index)++;
            }
        }
    }

    void DSD::P25readWord (char* word, unsigned int length, int* status_count, P25_Heuristics::P25_AnalogSignal* analog_signal_array, int* analog_signal_index) {
      P25readDibitUpdateAnalogData(word, length, status_count, analog_signal_array, analog_signal_index);
    }

    void DSD::P25readGolay24Parity (char* parity, int* status_count, P25_Heuristics::P25_AnalogSignal* analog_signal_array, int* analog_signal_index) {
      P25readDibitUpdateAnalogData(parity, 12, status_count, analog_signal_array, analog_signal_index);
    }

    void DSD::P25readHammParity (char* parity, int* status_count, P25_Heuristics::P25_AnalogSignal* analog_signal_array, int* analog_signal_index) {
      // Read 2 dibits = read 4 bits.
      P25readDibitUpdateAnalogData(parity, 4, status_count, analog_signal_array, analog_signal_index);
    }

    /**
    * Corrects a hex (6 bit) word  using the Golay 24 FEC.
    */
    void DSD::P25CorrectHexWord (char* hex, char* parity) {
      int fixed_errors;
      golay24.decode_6(hex, parity, &fixed_errors);
    }

    /**
    * Reverse the order of bits in a 12-bit word. We need this to accommodate to the expected bit order in
    * some algorithms.
    * \param dodeca The 12-bit word to reverse.
    */
    void DSD::P25swapHexWordsBits(char* dodeca) {
      int j;
      for(j=0; j<6; j++) {
          int swap;
          swap = dodeca[j];
          dodeca[j] = dodeca[j+6];
          dodeca[j+6] = swap;
        }
    }

    /**
    * Reverse the order of bits in a sequence of six pairs of 12-bit words and their parities.
    * \param dodeca_data Pointer to the start of the 12-bit words sequence.
    * \param dodeca_parity Pointer to the parities sequence.
    */
    void DSD::P25swapHexWords(char* dodeca_data, char* dodeca_parity) {
      int i;
      for(i=0; i<6; i++) {
          P25swapHexWordsBits(dodeca_data + i*12);
          P25swapHexWordsBits(dodeca_parity + i*12);
      }
    }

    /**
    * Reads an hex word, its parity bits and attempts to error correct it using the Golay24 algorithm.
    */
    void DSD::P25readAndCorrectHexWord (char* hex, int* status_count, P25_Heuristics::P25_AnalogSignal* analog_signal_array, int* analog_signal_index) {
      char parity[12];

      // Read the hex word
      P25readWord (hex, 6, status_count, analog_signal_array, analog_signal_index);
      // Read the parity
      P25readGolay24Parity (parity, status_count, analog_signal_array, analog_signal_index);
      // Use the Golay24 FEC to correct it. This call modifies the content of hex to fix it, hopefully
      P25CorrectHexWord (hex, parity);
    }

    void DSD::P25readAndCorrectHexWordHamming (char* hex, int* status_count, P25_Heuristics::P25_AnalogSignal* analog_signal_array, int* analog_signal_index) {
      char parity[4];
      // Read the hex word
      P25readWord (hex, 6, status_count, analog_signal_array, analog_signal_index);
      // Read the parity
      P25readHammParity (parity, status_count, analog_signal_array, analog_signal_index);
      // Use Hamming to error correct the hex word
      hamming.decode(hex, parity);
    }

    /**
    * Read an hex word, its parity bits and attempts to error correct it using the Hamming algorithm.
    */
    void DSD::P25readAndCorrectDodecaWord (char* dodeca, int* status_count, P25_Heuristics::P25_AnalogSignal* analog_signal_array, int* analog_signal_index) {
      char parity[12];
      int fixed_errors;
      int irrecoverable_errors;

      // Read the hex word
      P25readWord (dodeca, 12, status_count, analog_signal_array, analog_signal_index);
      // Read the parity
      P25readGolay24Parity (parity, status_count, analog_signal_array, analog_signal_index);

      // Use extended golay to error correct the dodeca word
      golay24.decode_12(dodeca, parity, &fixed_errors);
    }

    void DSD::P25correctHammingDibits(char* data, int count, P25_Heuristics::P25_AnalogSignal* analog_signal_array) {
      char parity[4];
      int i, j;
      int analog_signal_index;

      analog_signal_index = 0;

      for (i=count-1; i>=0; i--) {
          // Take the next 3 dibits (1 hex word) as they are
          for (j=0; j<6; j+=2) {
              // 3 iterations -> 3 dibits 
              int dibit = (data[i*6+j] << 1) | data[i*6+j+1];
              // This dibit is the correct value we should have read in the first place
              analog_signal_array[analog_signal_index].corrected_dibit = dibit;

              analog_signal_index++;
          }

          // The next two dibits are calculated has the hamming parity of the hex word
          hamming.encode(data+i*6, parity);

          for (j=0; j<4; j+=2) {
              // 2 iterations -> 2 dibits 
              int dibit = (parity[j] << 1) | parity[j+1];
              // Again, this dibit is the correct value we should have read in the first place
              analog_signal_array[analog_signal_index].corrected_dibit = dibit;

              analog_signal_index++;
          }
      }
    }

    /**
    * Uses the information from a corrected sequence of hex words to update the P25_Heuristics::P25_AnalogSignal data.
    * The proper Golay 24 parity is calculated from the corrected hex word so we can also fix the Golay parity
    * that we read originally from the signal.
    * \param corrected_hex_data Pointer to a sequence of hex words that has been error corrected and therefore
    * we trust it's correct. Typically this are hex words that has been decoded successfully using a
    * Reed-Solomon variant.
    * \param hex_count The number of hex words in the sequence.
    * \param analog_signal_array A pointer to the P25_Heuristics::P25_AnalogSignal information for the sequence of hex words.
    */
    void DSD::P25correctGolayDibits6(char* corrected_hex_data, int hex_count, P25_Heuristics::P25_AnalogSignal* analog_signal_array) {
      int i, j;
      int analog_signal_index;
      int dibit;
      char parity[12];
      analog_signal_index = 0;

      for (i=hex_count-1; i>=0; i--) {
          for (j=0; j<6; j+=2) {
              // 3 iterations -> 3 dibits 
              // Given the bits, calculates the dibit
              dibit = (corrected_hex_data[i*6+j] << 1) | corrected_hex_data[i*6+j+1];
              // Now we know the dibit we should have read from the signal
              analog_signal_array[analog_signal_index].corrected_dibit = dibit;

              analog_signal_index++;
          }

          // Calculate the Golay 24 parity for the corrected hex word
          golay24.encode_6(corrected_hex_data+i*6, parity);

          // Now we know the parity we should have read from the signal. Use this information
          for (j=0; j<12; j+=2) {
              // 6 iterations -> 6 dibits 
              // Given the bits, calculates the dibit
              dibit = (parity[j] << 1) | parity[j+1];
              // Now we know the dibit we should have read from the signal
              analog_signal_array[analog_signal_index].corrected_dibit = dibit;

              analog_signal_index++;
          }
      }
    }

    /**
    * Correct the information in analog_signal_array according with the content of data, which has been
    * error corrected and should be valid.
    * \param data A sequence of 12-bit words.
    * \param count Number of words in the sequence.
    * \param analog_signal_array Pointer to a sequence of P25_Heuristics::P25_AnalogSignal elements, as many as the value of count.
    */
    void DSD::P25correctGolayDibits12(char* data, int count, P25_Heuristics::P25_AnalogSignal* analog_signal_array) {
      int i, j;
      int analog_signal_index;
      int dibit;
      char parity[12];


      analog_signal_index = 0;

      for (i=count-1; i>=0; i--) {
          for (j=0; j<12; j+=2) {
              // 6 iterations -> 6 dibits 
              dibit = (data[i*12+j] << 1) | data[i*12+j+1];
              analog_signal_array[analog_signal_index].corrected_dibit = dibit;

              analog_signal_index++;
          }

          // Calculate the golay parity for the hex word
          golay24.encode_12(data+i*12, parity);

          for (j=0; j<12; j+=2) {
              // 6 iterations -> 6 dibits 
              dibit = (parity[j] << 1) | parity[j+1];
              analog_signal_array[analog_signal_index].corrected_dibit = dibit;

              analog_signal_index++;
          }
      }
    }

    void DSD::P25readZeros(P25_Heuristics::P25_AnalogSignal* analog_signal_array, unsigned int length, int* status_count, int new_sequence) {
      char buffer[length];
      unsigned int i;
      int analog_signal_index;

      analog_signal_index = 0;
      P25readDibitUpdateAnalogData (buffer, length, status_count, analog_signal_array, &analog_signal_index);
      if (new_sequence) {
          analog_signal_array[0].sequence_broken = 1;
      }

      for (i=0;i<length/2;i++) {
          analog_signal_array[i].corrected_dibit = 0;
      }

      // We know that all these bits should be zero. Use this information for the heuristics module
      p25Heuristics.contribute_to_heuristics(rfMod, analog_signal_array, length/2);
    }

    /**
    * The important method that processes a full P25 HD unit.
    */
    void DSD::P25processHDU() {
      char mi[73], mfid[9], algid[9], kid[17], tgid[17], tmpstr[255];
      int i, j;
      long talkgroup;
      int algidhex, kidhex;
      char hex[6];
      int status_count;
      int status;

      char hex_data[20][6];    // Data in hex-words (6 bit words). A total of 20 hex words.
      char hex_parity[16][6];  // Parity of the data, again in hex-word format. A total of 16 parity hex words.

      int irrecoverable_errors;

      P25_Heuristics::P25_AnalogSignal analog_signal_array[20*(3+6)+16*(3+6)];
      int analog_signal_index;

      analog_signal_index = 0;

      // we skip the status dibits that occur every 36 symbols
      // the next status symbol comes in 14 dibits from here
      // so we start counter at 36-14-1 = 21
      status_count = 21;

      // Read 20 hex words, correct them using their Golay 24 parity data.
      for (i=19; i>=0; i--) {
          P25readAndCorrectHexWord (hex, &status_count, analog_signal_array, &analog_signal_index);
          // Store the corrected hex word into the hex_data store:
          for (j=0; j<6; j++) {
              hex_data[i][j] = hex[j];
          }
      }

      // Read the 16 parity hex word. These are used to FEC the 20 hex words using Reed-Solomon.
      for (i=15; i>=0; i--) {
          P25readAndCorrectHexWord (hex, &status_count, analog_signal_array, &analog_signal_index);
          // Store the corrected hex word into the hex_parity store:
          for (j=0; j<6; j++) {
              hex_parity[i][j] = hex[j];
          }
      }
      // Don't forget to mark the first element as the start of a new sequence
      analog_signal_array[0].sequence_broken = 1;

      // Use the Reed-Solomon algorithm to correct the data. hex_data is modified in place
      irrecoverable_errors = reed_solomon_36_20_17.decode((char*)hex_data, (char*)hex_parity);
      if (irrecoverable_errors != 0) {
          // The hex words failed the Reed-Solomon check. There were too many errors. Still we can use this
          // information to update an estimate of the BER.
          //debug_header_critical_errors++;

          // We can correct (17-1)/2 = 8 errors. If we failed, it means that there were more than 8 errors in
          // these 20+16 words. But take into account that each hex word was already error corrected with
          // Golay 24, which can correct 3 bits on each sequence of (6+12) bits. We could say that there were
          // 9 errors of 4 bits.
          p25Heuristics.update_error_stats(20*6+16*6, 9*4);
      } else {
          // The hex words passed the Reed-Solomon check. This means that very likely they are correct and we
          // can trust that the digitizer did a good job with them. In other words, each analog value was
          // correctly assigned to a dibit. This is extremely useful information for the digitizer and we are
          // going to exploit it.
          char fixed_parity[16*6];

          // Correct the dibits that we did read according with the newly corrected hex_data values
          P25correctGolayDibits6((char*)hex_data, 20, analog_signal_array);

          // Generate again the Reed-Solomon parity for the corrected data
          reed_solomon_36_20_17.encode((char*)hex_data, fixed_parity);

          // Correct the dibits that we read according with the corrected parity values
          P25correctGolayDibits6(fixed_parity, 16, analog_signal_array+20*(3+6));

          // Now we have a bunch of dibits (composed of data and parity of different kinds) that we trust are all
          // correct. We also keep a record of the analog values from where each dibit is coming from.
          // This information is gold for the heuristics module.
          p25Heuristics.contribute_to_heuristics(rfMod, analog_signal_array, 20*(3+6)+16*(3+6));
      }

      // Now put the corrected data on the DSD structures

      mi[72] = 0;
      mfid[8] = 0;
      algid[8] = 0;
      kid[16] = 0;
      tgid[16] = 0;

      mi[ 0]   = hex_data[19][0] + '0';
      mi[ 1]   = hex_data[19][1] + '0';
      mi[ 2]   = hex_data[19][2] + '0';
      mi[ 3]   = hex_data[19][3] + '0';
      mi[ 4]   = hex_data[19][4] + '0';
      mi[ 5]   = hex_data[19][5] + '0';

      mi[ 6]   = hex_data[18][0] + '0';
      mi[ 7]   = hex_data[18][1] + '0';
      mi[ 8]   = hex_data[18][2] + '0';
      mi[ 9]   = hex_data[18][3] + '0';
      mi[10]   = hex_data[18][4] + '0';
      mi[11]   = hex_data[18][5] + '0';

      mi[12]   = hex_data[17][0] + '0';
      mi[13]   = hex_data[17][1] + '0';
      mi[14]   = hex_data[17][2] + '0';
      mi[15]   = hex_data[17][3] + '0';
      mi[16]   = hex_data[17][4] + '0';
      mi[17]   = hex_data[17][5] + '0';

      mi[18]   = hex_data[16][0] + '0';
      mi[19]   = hex_data[16][1] + '0';
      mi[20]   = hex_data[16][2] + '0';
      mi[21]   = hex_data[16][3] + '0';
      mi[22]   = hex_data[16][4] + '0';
      mi[23]   = hex_data[16][5] + '0';

      mi[24]   = hex_data[15][0] + '0';
      mi[25]   = hex_data[15][1] + '0';
      mi[26]   = hex_data[15][2] + '0';
      mi[27]   = hex_data[15][3] + '0';
      mi[28]   = hex_data[15][4] + '0';
      mi[29]   = hex_data[15][5] + '0';

      mi[30]   = hex_data[14][0] + '0';
      mi[31]   = hex_data[14][1] + '0';
      mi[32]   = hex_data[14][2] + '0';
      mi[33]   = hex_data[14][3] + '0';
      mi[34]   = hex_data[14][4] + '0';
      mi[35]   = hex_data[14][5] + '0';

      mi[36]   = hex_data[13][0] + '0';
      mi[37]   = hex_data[13][1] + '0';
      mi[38]   = hex_data[13][2] + '0';
      mi[39]   = hex_data[13][3] + '0';
      mi[40]   = hex_data[13][4] + '0';
      mi[41]   = hex_data[13][5] + '0';

      mi[42]   = hex_data[12][0] + '0';
      mi[43]   = hex_data[12][1] + '0';
      mi[44]   = hex_data[12][2] + '0';
      mi[45]   = hex_data[12][3] + '0';
      mi[46]   = hex_data[12][4] + '0';
      mi[47]   = hex_data[12][5] + '0';

      mi[48]   = hex_data[11][0] + '0';
      mi[49]   = hex_data[11][1] + '0';
      mi[50]   = hex_data[11][2] + '0';
      mi[51]   = hex_data[11][3] + '0';
      mi[52]   = hex_data[11][4] + '0';
      mi[53]   = hex_data[11][5] + '0';

      mi[54]   = hex_data[10][0] + '0';
      mi[55]   = hex_data[10][1] + '0';
      mi[56]   = hex_data[10][2] + '0';
      mi[57]   = hex_data[10][3] + '0';
      mi[58]   = hex_data[10][4] + '0';
      mi[59]   = hex_data[10][5] + '0';

      mi[60]   = hex_data[ 9][0] + '0';
      mi[61]   = hex_data[ 9][1] + '0';
      mi[62]   = hex_data[ 9][2] + '0';
      mi[63]   = hex_data[ 9][3] + '0';
      mi[64]   = hex_data[ 9][4] + '0';
      mi[65]   = hex_data[ 9][5] + '0';

      mi[66]   = hex_data[ 8][0] + '0';
      mi[67]   = hex_data[ 8][1] + '0';
      mi[68]   = hex_data[ 8][2] + '0';
      mi[69]   = hex_data[ 8][3] + '0';
      mi[70]   = hex_data[ 8][4] + '0';
      mi[71]   = hex_data[ 8][5] + '0';

      mfid[0]  = hex_data[ 7][0] + '0';
      mfid[1]  = hex_data[ 7][1] + '0';
      mfid[2]  = hex_data[ 7][2] + '0';
      mfid[3]  = hex_data[ 7][3] + '0';
      mfid[4]  = hex_data[ 7][4] + '0';
      mfid[5]  = hex_data[ 7][5] + '0';

      mfid[6]  = hex_data[ 6][0] + '0';
      mfid[7]  = hex_data[ 6][1] + '0';
      algid[0] = hex_data[ 6][2] + '0';  // The important algorithm ID. This indicates whether the data is
      algid[1] = hex_data[ 6][3] + '0';  // encrypted and if so what is the encryption algorithm used.
      algid[2] = hex_data[ 6][4] + '0';  // A code 0x80 here means that the data is unencrypted.
      algid[3] = hex_data[ 6][5] + '0';

      algid[4] = hex_data[ 5][0] + '0';
      algid[5] = hex_data[ 5][1] + '0';
      algid[6] = hex_data[ 5][2] + '0';
      algid[7] = hex_data[ 5][3] + '0';
      kid[ 0]  = hex_data[ 5][4] + '0';
      kid[ 1]  = hex_data[ 5][5] + '0';

      kid[ 2]  = hex_data[ 4][0] + '0';  // The encryption key ID
      kid[ 3]  = hex_data[ 4][1] + '0';
      kid[ 4]  = hex_data[ 4][2] + '0';
      kid[ 5]  = hex_data[ 4][3] + '0';
      kid[ 6]  = hex_data[ 4][4] + '0';
      kid[ 7]  = hex_data[ 4][5] + '0';

      kid[ 8]  = hex_data[ 3][0] + '0';
      kid[ 9]  = hex_data[ 3][1] + '0';
      kid[10]  = hex_data[ 3][2] + '0';
      kid[11]  = hex_data[ 3][3] + '0';
      kid[12]  = hex_data[ 3][4] + '0';
      kid[13]  = hex_data[ 3][5] + '0';

      kid[14]  = hex_data[ 2][0] + '0';
      kid[15]  = hex_data[ 2][1] + '0';
      tgid[ 0] = hex_data[ 2][2] + '0';  // Talk group ID
      tgid[ 1] = hex_data[ 2][3] + '0';
      tgid[ 2] = hex_data[ 2][4] + '0';
      tgid[ 3] = hex_data[ 2][5] + '0';

      tgid[ 4] = hex_data[ 1][0] + '0';
      tgid[ 5] = hex_data[ 1][1] + '0';
      tgid[ 6] = hex_data[ 1][2] + '0';
      tgid[ 7] = hex_data[ 1][3] + '0';
      tgid[ 8] = hex_data[ 1][4] + '0';
      tgid[ 9] = hex_data[ 1][5] + '0';

      tgid[10] = hex_data[ 0][0] + '0';
      tgid[11] = hex_data[ 0][1] + '0';
      tgid[12] = hex_data[ 0][2] + '0';
      tgid[13] = hex_data[ 0][3] + '0';
      tgid[14] = hex_data[ 0][4] + '0';
      tgid[15] = hex_data[ 0][5] + '0';

      p25kid = strtol(kid, NULL, 2);

      skipDibit (5);
      status = getDibit ();
      //TODO: Do something useful with the status bits...

      if (p25enc == 1) {
          algidhex = strtol (algid, NULL, 2);
          kidhex = strtol (kid, NULL, 2);
//          printf ("mi: %s algid: $%x kid: $%x\n", mi, algidhex, kidhex);
      }
      if (p25lc == 1) {
//          printf ("mfid: %s tgid: %s ", mfid, tgid);
          if (p25tg == 0) {
//              printf ("\n");
          }
      }

      j = 0;
      if (strcmp (mfid, "10010000") == 0) {
          for (i = 4; i < 16; i++) {
              if (tgcount < 24) {
                  tg[tgcount][j] = tgid[i];
              }
              tmpstr[j] = tgid[i];
              j++;
          }
          tmpstr[12] = '0';
          tmpstr[13] = '0';
          tmpstr[14] = '0';
          tmpstr[15] = '0';
      } else {
          for (i = 0; i < 16; i++) {
              if (tgcount < 24) {
                  tg[tgcount][j] = tgid[i];
              }
              tmpstr[j] = tgid[i];
              j++;
          }
      }
      tmpstr[16] = 0;
      talkgroup = strtol (tmpstr, NULL, 2);
      lasttg = talkgroup;
      if (tgcount < 24) {
          tgcount = tgcount + 1;
      }
      if (p25tg == 1) {
//          printf ("tg: %li\n", talkgroup);
      }
    }

    void DSD::P25processIMBE (int* status_count) {
      int j, dibit, status;
      char imbe_fr[8][23];
      const int *w, *x, *y, *z;

      w = iW;
      x = iX;
      y = iY;
      z = iZ;

      for (j = 0; j < 72; j++) {
          if (*status_count == 35) {
              // Skip the status symbol
              status = getDibit ();
              // TODO: do something useful with the status bits...
              *status_count = 1;
          } else {
              (*status_count)++;
          }
          dibit = getDibit ();
          imbe_fr[*w][*x] = (1 & (dibit >> 1)); // bit 1
          imbe_fr[*y][*z] = (1 & dibit);        // bit 0

          w++;
          x++;
          y++;
          z++;
      }

      if (p25kid == 0 || unmute_encrypted_p25 == 1) {
          // Check for a non-standard c0 transmitted
          // This is explained here: https://github.com/szechyjs/dsd/issues/24
          char non_standard_word[23] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0};
          int match = 1;
          unsigned int i;
          for (i=0; i<23; i++) {
              if (imbe_fr[0][i] != non_standard_word[i]) {
                  match = 0;
                  break;
              }
          }

          if (match) {
              // Skip this particular value. If we let it pass it will be signaled as an erroneus IMBE
//              printf("(Non-standard IMBE c0 detected, skipped)");
          } else {
              processMbeFrame (imbe_fr, NULL, NULL);
          }
      }
    }

    void DSD::P25processLDU1 () {
        // extracts IMBE frames from LDU frame
        int i;
        char lcformat[9], mfid[9], lcinfo[57];
        char lsd1[9], lsd2[9];

        int status_count;

        char hex_data[12][6];    // Data in hex-words (6 bit words). A total of 12 hex words.
        char hex_parity[12][6];  // Parity of the data, again in hex-word format. A total of 12 parity hex words.

        int irrecoverable_errors;

        P25_Heuristics::P25_AnalogSignal analog_signal_array[12*(3+2)+12*(3+2)];
        int analog_signal_index;


        analog_signal_index = 0;

        // we skip the status dibits that occur every 36 symbols
        // the first IMBE frame starts 14 symbols before next status
        // so we start counter at 36-14-1 = 21
        status_count = 21;

        if (errorbars == 1) {
            status_mbedecoding = true;
            status_errorbar = "";
//            printf ("e:");
        }

        // IMBE 1
        P25processIMBE (&status_count);

        // IMBE 2
        P25processIMBE (&status_count);

        // Read data after IMBE 2
        P25readAndCorrectHexWordHamming (&(hex_data[11][0]), &status_count, analog_signal_array, &analog_signal_index);
        P25readAndCorrectHexWordHamming (&(hex_data[10][0]), &status_count, analog_signal_array, &analog_signal_index);
        P25readAndCorrectHexWordHamming (&(hex_data[ 9][0]), &status_count, analog_signal_array, &analog_signal_index);
        P25readAndCorrectHexWordHamming (&(hex_data[ 8][0]), &status_count, analog_signal_array, &analog_signal_index);
        analog_signal_array[0*5].sequence_broken = 1;

        // IMBE 3
        P25processIMBE (&status_count);

        // Read data after IMBE 3
        P25readAndCorrectHexWordHamming (&(hex_data[ 7][0]), &status_count, analog_signal_array, &analog_signal_index);
        P25readAndCorrectHexWordHamming (&(hex_data[ 6][0]), &status_count, analog_signal_array, &analog_signal_index);
        P25readAndCorrectHexWordHamming (&(hex_data[ 5][0]), &status_count, analog_signal_array, &analog_signal_index);
        P25readAndCorrectHexWordHamming (&(hex_data[ 4][0]), &status_count, analog_signal_array, &analog_signal_index);
        analog_signal_array[4*5].sequence_broken = 1;

        // IMBE 4
        P25processIMBE (&status_count);

        // Read data after IMBE 4
        P25readAndCorrectHexWordHamming (&(hex_data[ 3][0]), &status_count, analog_signal_array, &analog_signal_index);
        P25readAndCorrectHexWordHamming (&(hex_data[ 2][0]), &status_count, analog_signal_array, &analog_signal_index);
        P25readAndCorrectHexWordHamming (&(hex_data[ 1][0]), &status_count, analog_signal_array, &analog_signal_index);
        P25readAndCorrectHexWordHamming (&(hex_data[ 0][0]), &status_count, analog_signal_array, &analog_signal_index);
        analog_signal_array[8*5].sequence_broken = 1;

        // IMBE 5
        P25processIMBE (&status_count);

        // Read data after IMBE 5
        P25readAndCorrectHexWordHamming (&(hex_parity[11][0]), &status_count, analog_signal_array, &analog_signal_index);
        P25readAndCorrectHexWordHamming (&(hex_parity[10][0]), &status_count, analog_signal_array, &analog_signal_index);
        P25readAndCorrectHexWordHamming (&(hex_parity[ 9][0]), &status_count, analog_signal_array, &analog_signal_index);
        P25readAndCorrectHexWordHamming (&(hex_parity[ 8][0]), &status_count, analog_signal_array, &analog_signal_index);
        analog_signal_array[12*5].sequence_broken = 1;

        // IMBE 6
        P25processIMBE (&status_count);

        // Read data after IMBE 6
        P25readAndCorrectHexWordHamming (&(hex_parity[ 7][0]), &status_count, analog_signal_array, &analog_signal_index);
        P25readAndCorrectHexWordHamming (&(hex_parity[ 6][0]), &status_count, analog_signal_array, &analog_signal_index);
        P25readAndCorrectHexWordHamming (&(hex_parity[ 5][0]), &status_count, analog_signal_array, &analog_signal_index);
        P25readAndCorrectHexWordHamming (&(hex_parity[ 4][0]), &status_count, analog_signal_array, &analog_signal_index);
        analog_signal_array[16*5].sequence_broken = 1;

        // IMBE 7
        P25processIMBE (&status_count);

        // Read data after IMBE 7
        P25readAndCorrectHexWordHamming (&(hex_parity[ 3][0]), &status_count, analog_signal_array, &analog_signal_index);
        P25readAndCorrectHexWordHamming (&(hex_parity[ 2][0]), &status_count, analog_signal_array, &analog_signal_index);
        P25readAndCorrectHexWordHamming (&(hex_parity[ 1][0]), &status_count, analog_signal_array, &analog_signal_index);
        P25readAndCorrectHexWordHamming (&(hex_parity[ 0][0]), &status_count, analog_signal_array, &analog_signal_index);
        analog_signal_array[20*5].sequence_broken = 1;

        // IMBE 8
        P25processIMBE (&status_count);

        // Read data after IMBE 8: LSD (low speed data)
        {
          char lsd[8];
          char cyclic_parity[8];

          for (i=0; i<=6; i+=2) {
              P25readDibit(lsd+i, &status_count, NULL, NULL);
          }
          for (i=0; i<=6; i+=2) {
              P25readDibit(cyclic_parity+i, &status_count, NULL, NULL);
          }
          for (i=0; i<8; i++) {
              lsd1[i] = lsd[i] + '0';
          }

          for (i=0; i<=6; i+=2) {
              P25readDibit(lsd+i, &status_count, NULL, NULL);
          }
          for (i=0; i<=6; i+=2) {
              P25readDibit(cyclic_parity+i, &status_count, NULL, NULL);
          }
          for (i=0; i<8; i++) {
              lsd2[i] = lsd[i] + '0';
          }

          // TODO: error correction of the LSD bytes...
          // TODO: do something useful with the LSD bytes...
        }

        // IMBE 9
        P25processIMBE (&status_count);

        if (errorbars == 1) {
            status_mbedecoding = false;
//            printf ("\n");
        }

        if (p25status == 1) {
//            printf ("lsd1: %s lsd2: %s\n", lsd1, lsd2);
        }

        // trailing status symbol
        {
            int status;
            status = getDibit () + '0';
            // TODO: do something useful with the status bits...
        }

        // Error correct the hex_data using Reed-Solomon hex_parity
        irrecoverable_errors = reed_solomon_24_12_13.decode((char*)hex_data, (char*)hex_parity);
        if (irrecoverable_errors == 1) {

            // We can correct (13-1)/2 = 6 errors. If we failed, it means that there were more than 6 errors in
            // these 12+12 words. But take into account that each hex word was already error corrected with
            // Hamming(10,6,3), which can correct 1 bits on each sequence of (6+4) bits. We could say that there
            // were 7 errors of 2 bits.
            p25Heuristics.update_error_stats(12*6+12*6, 7*2);
        } else {
            // Same comments as in processHDU. See there.

            char fixed_parity[12*6];

            // Correct the dibits that we read according with hex_data values
            P25correctHammingDibits((char*)hex_data, 12, analog_signal_array);

            // Generate again the Reed-Solomon parity
            reed_solomon_24_12_13.encode((char*)hex_data, fixed_parity);

            // Correct the dibits that we read according with the fixed parity values
            P25correctHammingDibits(fixed_parity, 12, analog_signal_array+12*(3+2));

            // Once corrected, contribute this information to the heuristics module
            p25Heuristics.contribute_to_heuristics(rfMod, analog_signal_array, 12*(3+2)+12*(3+2));
        }

        // Now put the corrected data into the DSD structures

        lcformat[8] = 0;
        mfid[8] = 0;
        lcinfo[56] = 0;
        lsd1[8] = 0;
        lsd2[8] = 0;

        lcformat[0] = hex_data[11][0] + '0';
        lcformat[1] = hex_data[11][1] + '0';
        lcformat[2] = hex_data[11][2] + '0';
        lcformat[3] = hex_data[11][3] + '0';
        lcformat[4] = hex_data[11][4] + '0';
        lcformat[5] = hex_data[11][5] + '0';

        lcformat[6] = hex_data[10][0] + '0';
        lcformat[7] = hex_data[10][1] + '0';
        mfid[0]     = hex_data[10][2] + '0';
        mfid[1]     = hex_data[10][3] + '0';
        mfid[2]     = hex_data[10][4] + '0';
        mfid[3]     = hex_data[10][5] + '0';

        mfid[4]     = hex_data[ 9][0] + '0';
        mfid[5]     = hex_data[ 9][1] + '0';
        mfid[6]     = hex_data[ 9][2] + '0';
        mfid[7]     = hex_data[ 9][3] + '0';
        lcinfo[0]   = hex_data[ 9][4] + '0';
        lcinfo[1]   = hex_data[ 9][5] + '0';

        lcinfo[2]   = hex_data[ 8][0] + '0';
        lcinfo[3]   = hex_data[ 8][1] + '0';
        lcinfo[4]   = hex_data[ 8][2] + '0';
        lcinfo[5]   = hex_data[ 8][3] + '0';
        lcinfo[6]   = hex_data[ 8][4] + '0';
        lcinfo[7]   = hex_data[ 8][5] + '0';

        lcinfo[8]   = hex_data[ 7][0] + '0';
        lcinfo[9]   = hex_data[ 7][1] + '0';
        lcinfo[10]  = hex_data[ 7][2] + '0';
        lcinfo[11]  = hex_data[ 7][3] + '0';
        lcinfo[12]  = hex_data[ 7][4] + '0';
        lcinfo[13]  = hex_data[ 7][5] + '0';

        lcinfo[14]  = hex_data[ 6][0] + '0';
        lcinfo[15]  = hex_data[ 6][1] + '0';
        lcinfo[16]  = hex_data[ 6][2] + '0';
        lcinfo[17]  = hex_data[ 6][3] + '0';
        lcinfo[18]  = hex_data[ 6][4] + '0';
        lcinfo[19]  = hex_data[ 6][5] + '0';

        lcinfo[20]  = hex_data[ 5][0] + '0';
        lcinfo[21]  = hex_data[ 5][1] + '0';
        lcinfo[22]  = hex_data[ 5][2] + '0';
        lcinfo[23]  = hex_data[ 5][3] + '0';
        lcinfo[24]  = hex_data[ 5][4] + '0';
        lcinfo[25]  = hex_data[ 5][5] + '0';

        lcinfo[26]  = hex_data[ 4][0] + '0';
        lcinfo[27]  = hex_data[ 4][1] + '0';
        lcinfo[28]  = hex_data[ 4][2] + '0';
        lcinfo[29]  = hex_data[ 4][3] + '0';
        lcinfo[30]  = hex_data[ 4][4] + '0';
        lcinfo[31]  = hex_data[ 4][5] + '0';

        lcinfo[32]  = hex_data[ 3][0] + '0';
        lcinfo[33]  = hex_data[ 3][1] + '0';
        lcinfo[34]  = hex_data[ 3][2] + '0';
        lcinfo[35]  = hex_data[ 3][3] + '0';
        lcinfo[36]  = hex_data[ 3][4] + '0';
        lcinfo[37]  = hex_data[ 3][5] + '0';

        lcinfo[38]  = hex_data[ 2][0] + '0';
        lcinfo[39]  = hex_data[ 2][1] + '0';
        lcinfo[40]  = hex_data[ 2][2] + '0';
        lcinfo[41]  = hex_data[ 2][3] + '0';
        lcinfo[42]  = hex_data[ 2][4] + '0';
        lcinfo[43]  = hex_data[ 2][5] + '0';

        lcinfo[44]  = hex_data[ 1][0] + '0';
        lcinfo[45]  = hex_data[ 1][1] + '0';
        lcinfo[46]  = hex_data[ 1][2] + '0';
        lcinfo[47]  = hex_data[ 1][3] + '0';
        lcinfo[48]  = hex_data[ 1][4] + '0';
        lcinfo[49]  = hex_data[ 1][5] + '0';

        lcinfo[50]  = hex_data[ 0][0] + '0';
        lcinfo[51]  = hex_data[ 0][1] + '0';
        lcinfo[52]  = hex_data[ 0][2] + '0';
        lcinfo[53]  = hex_data[ 0][3] + '0';
        lcinfo[54]  = hex_data[ 0][4] + '0';
        lcinfo[55]  = hex_data[ 0][5] + '0';

        P25processlcw (lcformat, mfid, lcinfo);
    }

    void DSD::P25processLDU2 () {
      // extracts IMBE frames from LDU frame
      int i;
      char mi[73], algid[9], kid[17];
      char lsd1[9], lsd2[9];
      int algidhex, kidhex;

      int status_count;

      char hex_data[16][6];    // Data in hex-words (6 bit words). A total of 16 hex words.
      char hex_parity[8][6];   // Parity of the data, again in hex-word format. A total of 12 parity hex words.

      int irrecoverable_errors;

      P25_Heuristics::P25_AnalogSignal analog_signal_array[16*(3+2)+8*(3+2)];
      int analog_signal_index;

      analog_signal_index = 0;

      // we skip the status dibits that occur every 36 symbols
      // the first IMBE frame starts 14 symbols before next status
      // so we start counter at 36-14-1 = 21
      status_count = 21;

      if (errorbars == 1) {
          status_mbedecoding = true;
          status_errorbar = "";
//          printf ("e:");
      }

      // IMBE 1
      P25processIMBE (&status_count);

      // IMBE 2
      P25processIMBE (&status_count);

      // Read data after IMBE 2
      P25readAndCorrectHexWordHamming (&(hex_data[15][0]), &status_count, analog_signal_array, &analog_signal_index);
      P25readAndCorrectHexWordHamming (&(hex_data[14][0]), &status_count, analog_signal_array, &analog_signal_index);
      P25readAndCorrectHexWordHamming (&(hex_data[13][0]), &status_count, analog_signal_array, &analog_signal_index);
      P25readAndCorrectHexWordHamming (&(hex_data[12][0]), &status_count, analog_signal_array, &analog_signal_index);
      analog_signal_array[0*5].sequence_broken = 1;

      // IMBE 3
      P25processIMBE (&status_count);

      // Read data after IMBE 3
      P25readAndCorrectHexWordHamming (&(hex_data[11][0]), &status_count, analog_signal_array, &analog_signal_index);
      P25readAndCorrectHexWordHamming (&(hex_data[10][0]), &status_count, analog_signal_array, &analog_signal_index);
      P25readAndCorrectHexWordHamming (&(hex_data[ 9][0]), &status_count, analog_signal_array, &analog_signal_index);
      P25readAndCorrectHexWordHamming (&(hex_data[ 8][0]), &status_count, analog_signal_array, &analog_signal_index);
      analog_signal_array[4*5].sequence_broken = 1;

      // IMBE 4
      P25processIMBE (&status_count);

      // Read data after IMBE 4
      P25readAndCorrectHexWordHamming (&(hex_data[ 7][0]), &status_count, analog_signal_array, &analog_signal_index);
      P25readAndCorrectHexWordHamming (&(hex_data[ 6][0]), &status_count, analog_signal_array, &analog_signal_index);
      P25readAndCorrectHexWordHamming (&(hex_data[ 5][0]), &status_count, analog_signal_array, &analog_signal_index);
      P25readAndCorrectHexWordHamming (&(hex_data[ 4][0]), &status_count, analog_signal_array, &analog_signal_index);
      analog_signal_array[8*5].sequence_broken = 1;

      // IMBE 5
      P25processIMBE (&status_count);

      // Read data after IMBE 5
      P25readAndCorrectHexWordHamming (&(hex_data[ 3][0]), &status_count, analog_signal_array, &analog_signal_index);
      P25readAndCorrectHexWordHamming (&(hex_data[ 2][0]), &status_count, analog_signal_array, &analog_signal_index);
      P25readAndCorrectHexWordHamming (&(hex_data[ 1][0]), &status_count, analog_signal_array, &analog_signal_index);
      P25readAndCorrectHexWordHamming (&(hex_data[ 0][0]), &status_count, analog_signal_array, &analog_signal_index);
      analog_signal_array[12*5].sequence_broken = 1;

      // IMBE 6
      P25processIMBE (&status_count);

      // Read data after IMBE 6
      P25readAndCorrectHexWordHamming (&(hex_parity[ 7][0]), &status_count, analog_signal_array, &analog_signal_index);
      P25readAndCorrectHexWordHamming (&(hex_parity[ 6][0]), &status_count, analog_signal_array, &analog_signal_index);
      P25readAndCorrectHexWordHamming (&(hex_parity[ 5][0]), &status_count, analog_signal_array, &analog_signal_index);
      P25readAndCorrectHexWordHamming (&(hex_parity[ 4][0]), &status_count, analog_signal_array, &analog_signal_index);
      analog_signal_array[16*5].sequence_broken = 1;

      // IMBE 7
      P25processIMBE (&status_count);

      // Read data after IMBE 7
      P25readAndCorrectHexWordHamming (&(hex_parity[ 3][0]), &status_count, analog_signal_array, &analog_signal_index);
      P25readAndCorrectHexWordHamming (&(hex_parity[ 2][0]), &status_count, analog_signal_array, &analog_signal_index);
      P25readAndCorrectHexWordHamming (&(hex_parity[ 1][0]), &status_count, analog_signal_array, &analog_signal_index);
      P25readAndCorrectHexWordHamming (&(hex_parity[ 0][0]), &status_count, analog_signal_array, &analog_signal_index);
      analog_signal_array[20*5].sequence_broken = 1;

      // IMBE 8
      P25processIMBE (&status_count);

      // Read data after IMBE 8: LSD (low speed data)
      {
        char lsd[8];
        char cyclic_parity[8];

        for (i=0; i<=6; i+=2) {
            P25readDibit(lsd+i, &status_count, NULL, NULL);
        }
        for (i=0; i<=6; i+=2) {
            P25readDibit(cyclic_parity+i, &status_count, NULL, NULL);
        }
        for (i=0; i<8; i++) {
            lsd1[i] = lsd[i] + '0';
        }

        for (i=0; i<=6; i+=2) {
            P25readDibit(lsd+i, &status_count, NULL, NULL);
        }
        for (i=0; i<=6; i+=2) {
            P25readDibit(cyclic_parity+i, &status_count, NULL, NULL);
        }
        for (i=0; i<8; i++) {
            lsd2[i] = lsd[i] + '0';
        }

        // TODO: error correction of the LSD bytes...
        // TODO: do something useful with the LSD bytes...
      }

      // IMBE 9
      P25processIMBE (&status_count);

      if (errorbars == 1) {
          status_mbedecoding = false;
//          printf ("\n");
      }

      if (p25status == 1) {
//          printf ("lsd1: %s lsd2: %s\n", lsd1, lsd2);
      }

      // trailing status symbol
      {
          int status;
          status = getDibit () + '0';
          // TODO: do something useful with the status bits...
      }

      // Error correct the hex_data using Reed-Solomon hex_parity
      irrecoverable_errors = reed_solomon_24_16_9.decode((char*)hex_data, (char*)hex_parity);
      if (irrecoverable_errors == 1) {

          // We can correct (9-1)/2 = 4 errors. If we failed, it means that there were more than 4 errors in
          // these 12+12 words. But take into account that each hex word was already error corrected with
          // Hamming(10,6,3), which can correct 1 bits on each sequence of (6+4) bits. We could say that there
          // were 5 errors of 2 bits.
          p25Heuristics.update_error_stats(12*6+12*6, 5*2);
        } else {
          // Same comments as in processHDU. See there.

          char fixed_parity[8*6];

          // Correct the dibits that we read according with hex_data values
          P25correctHammingDibits((char*)hex_data, 16, analog_signal_array);

          // Generate again the Reed-Solomon parity
          reed_solomon_24_16_9.encode((char*)hex_data, fixed_parity);

          // Correct the dibits that we read according with the fixed parity values
          P25correctHammingDibits(fixed_parity, 8, analog_signal_array+16*(3+2));

          // Once corrected, contribute this information to the heuristics module
          p25Heuristics.contribute_to_heuristics(rfMod, analog_signal_array, 16*(3+2)+8*(3+2));
        }

      // Now put the corrected data into the DSD structures

      mi[72] = 0;
      algid[8] = 0;
      kid[16] = 0;
      lsd1[8] = 0;
      lsd2[8] = 0;

      mi[ 0]   = hex_data[15][0] + '0';
      mi[ 1]   = hex_data[15][1] + '0';
      mi[ 2]   = hex_data[15][2] + '0';
      mi[ 3]   = hex_data[15][3] + '0';
      mi[ 4]   = hex_data[15][4] + '0';
      mi[ 5]   = hex_data[15][5] + '0';

      mi[ 6]   = hex_data[14][0] + '0';
      mi[ 7]   = hex_data[14][1] + '0';
      mi[ 8]   = hex_data[14][2] + '0';
      mi[ 9]   = hex_data[14][3] + '0';
      mi[10]   = hex_data[14][4] + '0';
      mi[11]   = hex_data[14][5] + '0';

      mi[12]   = hex_data[13][0] + '0';
      mi[13]   = hex_data[13][1] + '0';
      mi[14]   = hex_data[13][2] + '0';
      mi[15]   = hex_data[13][3] + '0';
      mi[16]   = hex_data[13][4] + '0';
      mi[17]   = hex_data[13][5] + '0';

      mi[18]   = hex_data[12][0] + '0';
      mi[19]   = hex_data[12][1] + '0';
      mi[20]   = hex_data[12][2] + '0';
      mi[21]   = hex_data[12][3] + '0';
      mi[22]   = hex_data[12][4] + '0';
      mi[23]   = hex_data[12][5] + '0';

      mi[24]   = hex_data[11][0] + '0';
      mi[25]   = hex_data[11][1] + '0';
      mi[26]   = hex_data[11][2] + '0';
      mi[27]   = hex_data[11][3] + '0';
      mi[28]   = hex_data[11][4] + '0';
      mi[29]   = hex_data[11][5] + '0';

      mi[30]   = hex_data[10][0] + '0';
      mi[31]   = hex_data[10][1] + '0';
      mi[32]   = hex_data[10][2] + '0';
      mi[33]   = hex_data[10][3] + '0';
      mi[34]   = hex_data[10][4] + '0';
      mi[35]   = hex_data[10][5] + '0';

      mi[36]   = hex_data[ 9][0] + '0';
      mi[37]   = hex_data[ 9][1] + '0';
      mi[38]   = hex_data[ 9][2] + '0';
      mi[39]   = hex_data[ 9][3] + '0';
      mi[40]   = hex_data[ 9][4] + '0';
      mi[41]   = hex_data[ 9][5] + '0';

      mi[42]   = hex_data[ 8][0] + '0';
      mi[43]   = hex_data[ 8][1] + '0';
      mi[44]   = hex_data[ 8][2] + '0';
      mi[45]   = hex_data[ 8][3] + '0';
      mi[46]   = hex_data[ 8][4] + '0';
      mi[47]   = hex_data[ 8][5] + '0';

      mi[48]   = hex_data[ 7][0] + '0';
      mi[49]   = hex_data[ 7][1] + '0';
      mi[50]   = hex_data[ 7][2] + '0';
      mi[51]   = hex_data[ 7][3] + '0';
      mi[52]   = hex_data[ 7][4] + '0';
      mi[53]   = hex_data[ 7][5] + '0';

      mi[54]   = hex_data[ 6][0] + '0';
      mi[55]   = hex_data[ 6][1] + '0';
      mi[56]   = hex_data[ 6][2] + '0';
      mi[57]   = hex_data[ 6][3] + '0';
      mi[58]   = hex_data[ 6][4] + '0';
      mi[59]   = hex_data[ 6][5] + '0';

      mi[60]   = hex_data[ 5][0] + '0';
      mi[61]   = hex_data[ 5][1] + '0';
      mi[62]   = hex_data[ 5][2] + '0';
      mi[63]   = hex_data[ 5][3] + '0';
      mi[64]   = hex_data[ 5][4] + '0';
      mi[65]   = hex_data[ 5][5] + '0';

      mi[66]   = hex_data[ 4][0] + '0';
      mi[67]   = hex_data[ 4][1] + '0';
      mi[68]   = hex_data[ 4][2] + '0';
      mi[69]   = hex_data[ 4][3] + '0';
      mi[70]   = hex_data[ 4][4] + '0';
      mi[71]   = hex_data[ 4][5] + '0';

      algid[0] = hex_data[ 3][0] + '0';
      algid[1] = hex_data[ 3][1] + '0';
      algid[2] = hex_data[ 3][2] + '0';
      algid[3] = hex_data[ 3][3] + '0';
      algid[4] = hex_data[ 3][4] + '0';
      algid[5] = hex_data[ 3][5] + '0';

      algid[6] = hex_data[ 2][0] + '0';
      algid[7] = hex_data[ 2][1] + '0';
      kid[0]   = hex_data[ 2][2] + '0';
      kid[1]   = hex_data[ 2][3] + '0';
      kid[2]   = hex_data[ 2][4] + '0';
      kid[3]   = hex_data[ 2][5] + '0';

      kid[4]   = hex_data[ 1][0] + '0';
      kid[5]   = hex_data[ 1][1] + '0';
      kid[6]   = hex_data[ 1][2] + '0';
      kid[7]   = hex_data[ 1][3] + '0';
      kid[8]   = hex_data[ 1][4] + '0';
      kid[9]   = hex_data[ 1][5] + '0';

      kid[10]  = hex_data[ 0][0] + '0';
      kid[11]  = hex_data[ 0][1] + '0';
      kid[12]  = hex_data[ 0][2] + '0';
      kid[13]  = hex_data[ 0][3] + '0';
      kid[14]  = hex_data[ 0][4] + '0';
      kid[15]  = hex_data[ 0][5] + '0';


      if (p25enc == 1) {
          algidhex = strtol (algid, NULL, 2);
          kidhex = strtol (kid, NULL, 2);
//          printf ("mi: %s algid: $%x kid: $%x\n", mi, algidhex, kidhex);
      }
    }

    void DSD::P25processTDULC () {
      int i;
      char lcinfo[57], lcformat[9], mfid[9];

      int status_count;

      char dodeca_data[6][12];    // Data in 12-bit words. A total of 6 words.
      char dodeca_parity[6][12];  // Reed-Solomon parity of the data. A total of 6 parity 12-bit words.

      int irrecoverable_errors;

      P25_Heuristics::P25_AnalogSignal analog_signal_array[6*(6+6)+6*(6+6)+10];
      int analog_signal_index;


      analog_signal_index = 0;

      // we skip the status dibits that occur every 36 symbols
      // the first IMBE frame starts 14 symbols before next status
      // so we start counter at 36-14-1 = 21
      status_count = 21;

      for(i=5; i>=0; i--) {
          P25readAndCorrectDodecaWord (&(dodeca_data[i][0]), &status_count, analog_signal_array, &analog_signal_index);
      }

      for(i=5; i>=0; i--) {
          P25readAndCorrectDodecaWord (&(dodeca_parity[i][0]), &status_count, analog_signal_array, &analog_signal_index);
      }

      // Swap the two 6-bit words to accommodate for the expected word order of the Reed-Solomon decoding
      P25swapHexWords((char*)dodeca_data, (char*)dodeca_parity);

      // Error correct the hex_data using Reed-Solomon hex_parity
      irrecoverable_errors = reed_solomon_24_12_13.decode((char*)dodeca_data, (char*)dodeca_parity);

      // Recover the original order
      P25swapHexWords((char*)dodeca_data, (char*)dodeca_parity);

      if (irrecoverable_errors == 1) {

          // We can correct (13-1)/2 = 6 errors. If we failed, it means that there were more than 6 errors in
          // these 12+12 words. But take into account that each hex word was already error corrected with
          // Golay 24, which can correct 3 bits on each sequence of (12+12) bits. We could say that there were
          // 7 errors of 4 bits.
          p25Heuristics.update_error_stats(12*6+12*6, 7*4);

      } else {
          // Same comments as in processHDU. See there.

          char fixed_parity[6*12];

          // Correct the dibits that we read according with hex_data values
          P25correctGolayDibits12((char*)dodeca_data, 6, analog_signal_array);

          // Generate again the Reed-Solomon parity
          // Now, swap again for Reed-Solomon
          P25swapHexWords((char*)dodeca_data, (char*)dodeca_parity);
          reed_solomon_24_12_13.encode((char*)dodeca_data, fixed_parity);
          // Swap again to recover the original order
          P25swapHexWords((char*)dodeca_data, fixed_parity);

          // Correct the dibits that we read according with the fixed parity values
          P25correctGolayDibits12(fixed_parity, 6, analog_signal_array+6*(6+6));

          // Once corrected, contribute this information to the heuristics module
          analog_signal_array[0].sequence_broken = 1;
          p25Heuristics.contribute_to_heuristics(rfMod, analog_signal_array, 6*(6+6)+6*(6+6));
      }


      // Next 10 dibits should be zeros
      // If an irrecoverable error happens, you should start a new sequence since the previous dibit was not set
      P25readZeros(analog_signal_array + 6*(6+6)+6*(6+6), 20, &status_count, irrecoverable_errors);

      // Next we should find an status dibit
      if (status_count != 35) {
//          printf("*** SYNC ERROR\n");
      }

      // trailing status symbol
      {
          int status;
          status = getDibit () + '0';
          // TODO: do something useful with the status bits...
      }

      // Put the corrected data into the DSD structures

      lcformat[8] = 0;
      mfid[8] = 0;
      lcinfo[56] = 0;

      lcformat[0] = dodeca_data[5][ 0] + '0';
      lcformat[1] = dodeca_data[5][ 1] + '0';
      lcformat[2] = dodeca_data[5][ 2] + '0';
      lcformat[3] = dodeca_data[5][ 3] + '0';
      lcformat[4] = dodeca_data[5][ 4] + '0';
      lcformat[5] = dodeca_data[5][ 5] + '0';
      lcformat[6] = dodeca_data[5][ 6] + '0';
      lcformat[7] = dodeca_data[5][ 7] + '0';
      mfid[0]     = dodeca_data[5][ 8] + '0';
      mfid[1]     = dodeca_data[5][ 9] + '0';
      mfid[2]     = dodeca_data[5][10] + '0';
      mfid[3]     = dodeca_data[5][11] + '0';

      mfid[4]     = dodeca_data[4][ 0] + '0';
      mfid[5]     = dodeca_data[4][ 1] + '0';
      mfid[6]     = dodeca_data[4][ 2] + '0';
      mfid[7]     = dodeca_data[4][ 3] + '0';
      lcinfo[0]   = dodeca_data[4][ 4] + '0';
      lcinfo[1]   = dodeca_data[4][ 5] + '0';
      lcinfo[2]   = dodeca_data[4][ 6] + '0';
      lcinfo[3]   = dodeca_data[4][ 7] + '0';
      lcinfo[4]   = dodeca_data[4][ 8] + '0';
      lcinfo[5]   = dodeca_data[4][ 9] + '0';
      lcinfo[6]   = dodeca_data[4][10] + '0';
      lcinfo[7]   = dodeca_data[4][11] + '0';

      lcinfo[8]   = dodeca_data[3][ 0] + '0';
      lcinfo[9]   = dodeca_data[3][ 1] + '0';
      lcinfo[10]  = dodeca_data[3][ 2] + '0';
      lcinfo[11]  = dodeca_data[3][ 3] + '0';
      lcinfo[12]  = dodeca_data[3][ 4] + '0';
      lcinfo[13]  = dodeca_data[3][ 5] + '0';
      lcinfo[14]  = dodeca_data[3][ 6] + '0';
      lcinfo[15]  = dodeca_data[3][ 7] + '0';
      lcinfo[16]  = dodeca_data[3][ 8] + '0';
      lcinfo[17]  = dodeca_data[3][ 9] + '0';
      lcinfo[18]  = dodeca_data[3][10] + '0';
      lcinfo[19]  = dodeca_data[3][11] + '0';

      lcinfo[20]  = dodeca_data[2][ 0] + '0';
      lcinfo[21]  = dodeca_data[2][ 1] + '0';
      lcinfo[22]  = dodeca_data[2][ 2] + '0';
      lcinfo[23]  = dodeca_data[2][ 3] + '0';
      lcinfo[24]  = dodeca_data[2][ 4] + '0';
      lcinfo[25]  = dodeca_data[2][ 5] + '0';
      lcinfo[26]  = dodeca_data[2][ 6] + '0';
      lcinfo[27]  = dodeca_data[2][ 7] + '0';
      lcinfo[28]  = dodeca_data[2][ 8] + '0';
      lcinfo[29]  = dodeca_data[2][ 9] + '0';
      lcinfo[30]  = dodeca_data[2][10] + '0';
      lcinfo[31]  = dodeca_data[2][11] + '0';

      lcinfo[32]  = dodeca_data[1][ 0] + '0';
      lcinfo[33]  = dodeca_data[1][ 1] + '0';
      lcinfo[34]  = dodeca_data[1][ 2] + '0';
      lcinfo[35]  = dodeca_data[1][ 3] + '0';
      lcinfo[36]  = dodeca_data[1][ 4] + '0';
      lcinfo[37]  = dodeca_data[1][ 5] + '0';
      lcinfo[38]  = dodeca_data[1][ 6] + '0';
      lcinfo[39]  = dodeca_data[1][ 7] + '0';
      lcinfo[40]  = dodeca_data[1][ 8] + '0';
      lcinfo[41]  = dodeca_data[1][ 9] + '0';
      lcinfo[42]  = dodeca_data[1][10] + '0';
      lcinfo[43]  = dodeca_data[1][11] + '0';

      lcinfo[44]  = dodeca_data[0][ 0] + '0';
      lcinfo[45]  = dodeca_data[0][ 1] + '0';
      lcinfo[46]  = dodeca_data[0][ 2] + '0';
      lcinfo[47]  = dodeca_data[0][ 3] + '0';
      lcinfo[48]  = dodeca_data[0][ 4] + '0';
      lcinfo[49]  = dodeca_data[0][ 5] + '0';
      lcinfo[50]  = dodeca_data[0][ 6] + '0';
      lcinfo[51]  = dodeca_data[0][ 7] + '0';
      lcinfo[52]  = dodeca_data[0][ 8] + '0';
      lcinfo[53]  = dodeca_data[0][ 9] + '0';
      lcinfo[54]  = dodeca_data[0][10] + '0';
      lcinfo[55]  = dodeca_data[0][11] + '0';

      P25processlcw (lcformat, mfid, lcinfo);
    }

    void DSD::P25processTDU () {
        P25_Heuristics::P25_AnalogSignal analog_signal_array[14];
        int status_count;

        // we skip the status dibits that occur every 36 symbols
        // the first IMBE frame starts 14 symbols before next status
        // so we start counter at 36-14-1 = 21
        status_count = 21;

        // Next 14 dibits should be zeros
        P25readZeros(analog_signal_array, 28, &status_count, 1);

        // Next we should find an status dibit
        if (status_count != 35) {
//            printf("*** SYNC ERROR\n");
        }

        // trailing status symbol
        {
            int status;
            status = getDibit () + '0';
            // TODO: do something useful with the status bits...
        }
    }
}
