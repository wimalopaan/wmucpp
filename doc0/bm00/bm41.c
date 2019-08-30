#include <avr/pgmspace.h>
#include <stdint.h>
#include <stdbool.h>

struct Note {
    uint16_t pitch;  
    uint16_t length;  
};

typedef struct Note Note_t;

static inline Note_t make_note(uint16_t p, uint16_t l) {
    Note_t n;
    n.pitch = p;
    n.length = l;
    return n;
}

#define SIZE 10

const Note_t array[10] PROGMEM = {{100, 200}, {200, 300}, {300, 400}, {400, 500}, {500, 600}, {100, 200}, {200, 300}, {300, 400}, {400, 500}, {500, 600}}; 
//const Note_t array2[5] PROGMEM = {{100, 200}, {200, 300}, {300, 400}, {400, 500}, {500, 600}}; 

volatile uint16_t r1;
volatile uint16_t r2;

int main() {
    uint8_t index = 0;
    
    while(true) {
        Note_t n = make_note(pgm_read_word(&(array[index].pitch)), pgm_read_word(&(array[index].length)));
        r1 = n.pitch;
        r2 = n.length;
        
        index = (++index == SIZE) ? 0 : index;
        
//        for(const Note_t* it = &array[0]; it < &array[10]; ++it) {
//            Note_t n = make_note(pgm_read_word(&it->pitch), pgm_read_word(&it->length));
            
//            r1 = n.pitch;
//            r2 = n.length;
//        }
        
    }
}
