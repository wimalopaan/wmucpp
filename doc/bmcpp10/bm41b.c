#include <stdio.h>
#include <stdbool.h>

struct Ressource {
    uint8_t data;
    bool    ok;
};
typedef struct Ressource ressource_t;

volatile ressource_t channel;

static inline int getchar_l() {
    if (channel.ok) {
        return channel.data;
    }
    return EOF;
} 

static inline int putchar_l(int data) {
    channel.data = data;
    if (channel.ok) {
        return data;
    }
    return EOF;
}
int main(){
    int c = EOF;
    while ((c = getchar_l()) != EOF) {
        if (putchar_l(c) == EOF) {
            return 1;
        }
    }
    return 0;
}
