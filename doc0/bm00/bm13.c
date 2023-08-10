#include <stdint.h>
#include <avr/pgmspace.h>

//const char s1[] PROGMEM = "abc";
//const char s2[] PROGMEM = "def";

//const char* const strings[] PROGMEM = {s1, s2};

//const char string_table[2][10] PROGMEM = {{"abc"}, {"def"}};

//const unsigned char displayMenus[7][12][18] PROGMEM = {
//  {{"Kurzschlusszeit"}, {"Stromgrenze"}, {"Stufenkurzschl."}, {"Abtastzeit"}, {"Weiche RCN-213"}, {"Kurze Lokaddr."}, {"RailCom BiDiB"}, {"Umschaltzeit"}, {"Neustarten"}, {"Sprache"}, {"Hilfe"}, {"Version"}},
//  {{"Short time"}, {"Current limit"}, {"Stage shorts"}, {"Sampling"}, {"Switch RCN-213"}, {"Short locoadress"}, {"RailCom BiDiB"}, {"Switchover"}, {"Short restart"}, {"Language"}, {"Help"}, {"Version"}},
//  {{"Korte tijd"}, {"Huidige limiet"}, {"Stappen kort"}, {"Monsterneming"}, {"Wissel RCN-213"}, {"Kort locadres"}, {"RailCom BiDiB"}, {"Omschakeling"}, {"Opnieuw te starten"}, {"Taal"}, {"Help"}, {"Versie"}},
//  {{"Kortslutning"}, {"Nuvaeren graense"}, {"Trin korte"}, {"Proveudtagning"}, {"Skift RCN-213"}, {"Kort adresse"}, {"RailCom BiDiB"}, {"Overgangen"}, {"Genstarte"}, {"Sprog"}, {"Hjaelp"}, {"Udgave"}},
//  {{"Tiempo corto"}, {"Lim de corriente"}, {"Pasos cortos"}, {"Muestreo"}, {"Cambiar RCN-213"}, {"loco. corta"}, {"RailCom BiDiB"}, {"Commutacion"}, {"Reiniciar"}, {"Lengua"}, {"Ayuder"}, {"Version"}},
//  {{"Temps court"}, {"Lim de courant"}, {"Les etapes court"}, {"Echantillonnage"}, {"Changer RCN-213"}, {"Adresse courte"}, {"RailCom BiDiB"}, {"Commutation"}, {"Redemarrage"}, {"Langue"}, {"Aider"}, {"Version"}},
//  {{"Orario ridotto"}, {"Limite di corr."}, {"Piccoli passi"}, {"Campionatura"}, {"Interru. RCN-213"}, {"Breve ind. loco."}, {"RailCom BiDiB"}, {"Commutazione"}, {"Ripresa"}, {"Lingua"}, {"Aiuto"}, {"Versione"}}
//};

//void foo(const char (*p)[500]) {
//}

//void bar(const char* p) {
//}



typedef struct {
	int a;
	int b;
} element_t;

typedef struct {
	element_t* const element;
	const size_t anzahl;
} config_t;

extern const config_t config;


struct S1 {
    int x;
};


void S1() {
}

void foo(config_t c) {
    
}

#include <stdfix.h>

int main() {
    unsigned _Sat _Accum a;
    
    
     element_t e1;
     element_t e2;
    
     e1 = e2;
    
     config_t c1;
     config_t c2;
     config_t c3 = c1;
     
//     c1 = c2;
     
    struct S1 s1 = {}; // Disambiguation: S1 s1; funktioniert nicht mehr, wenn Funktion S1() deklariert ist
    return s1.x;
}

