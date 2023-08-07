#include "../include/read_gcode.h"

/** Helper function converting string to float
 *  * */
static float strto_f(char* string, uint8_t* length){
    float number = 0;
    bool positive = true;
    float div = 0.f;
    char c;
    while((c = *string) != 0){
        if(c == ' ' || c == '\n' || c == '\r'){
            break;
        }
        else if(c == '-'){
            positive = false;
        }
        else if(c == '.'){
            div = 1.f;
        }
        else if(c >= '0' && c <= '9'){
            if(div == 0.f){
                number = number*10.f + (float)(c-'0');
            }
            else{
                div *= 10.f;
                number += (float)(c-'0')/div;
            }
        }
        string++;
        (*length)++;
    }
    if(positive == false){
        number *= -1;
    }
    return number;
}

static uint8_t read_float(const char* line, float* value, uint16_t* char_counter){
    char* number_start = const_cast<char *>(line + *char_counter);
    uint8_t length = 0;

    (*value) = strto_f(number_start, &length);

    (*char_counter) += length;
    if(line[*char_counter] == '\n' || line[*char_counter] == '\r' || line[*char_counter] == '\0'
       || line[*char_counter] == ';'){
        return 0;
    }
    else{
        return 1;
    }
}

/** @brief Parses message and loads command/parameter letter and associated with it value into proper variables
 *
 * @param letter    - Pointer to the char containing the command/parameter letter, to be updated in the function
 * @param value     - Pointer to the float containing the value of the command/parameter, to be updated in the function
 * @param line      - Whole GCODE command line, to be parsed
 * @param char_counter - Pointer to the uint16_t variable containing the number of characters that have been
 *                       already parsed
 *
 * @returns uint8_t value\n
 *          If equal to 0, there is no more commands/parameters to be parsed
 *          Otherwise equal to 1.
 *
 * */
uint8_t parseMessage(char* letter, float* value, const char* line, uint16_t* char_counter){
    while (line[*char_counter] == ' ') (*char_counter)++;

    if(line[*char_counter] == '\n' || line[*char_counter] == '\r' || line[*char_counter] == '\0'
       || line[*char_counter] == ';'){
        return 0;
    }

    *letter = line[*char_counter];
    (*char_counter)++;
    return read_float(line, value, char_counter);
}
