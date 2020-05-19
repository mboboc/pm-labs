### Lab00:

**Microcontroller** 
* procesor + memorie(volatila/nevolatila) + i/o
* opereaza la frecvente reduse (zeci-sute MHz)
* cost de productie mic si consum redus

structura:
* CPU
* RAM/FLASH/EEPROM
* GPIO
* USART/SPI/I2C
* timere
* ADC
* interfata DEBUG

periferic = orice dispozitiv, intern sau extern, care se conectează la un sistem de calcul și îi extinde funcționalitatea de bază

**Familia AVR** 
- arhitectura Harvard
- memoriile Flash (nonvolatila - stocheaza instructiuni), EEPROM, şi SRAM sunt integrate în acelaşi chip
- pipeline cu 2 nivele

ATMEGA324
- 8biti
- 32 KB Flash
- 1 KB EEPROM
- 2 KB RAM
- 20 MHz frecvența maximă de lucru
- tensiune de alimentare între 1.8V și 5.5 V
- 6 canale de PWM
- 8 canale de multiplexare ADC rez = 10biti (un singur ADC si se face multiplexare) --> o singura conversie la un moment de timp
- 4 porturi GPIO (fiecare cu 8 pini, în total 32 de pini) P-ABCD
- 3 timere (două pe 8 biți și unul pe 16 biți)
- interfeţe de comunicație seriale: USART, SPI, TWI
- interfaţe de programare ISP și debug JTAG
- DIP40

<insert poza cu pini>

**Secventa de compilare**
<poza secventa de compilare>

**Programarea microcontrollerului**
- folosind un ISP
- folosind bootloader (program incarcat la sfarsitul memoriei de program; folosirea lui presupune indeplinirea unei conditii care poate fi programata - un buton este apasat sau nu)

Pentru incarcarea programului in memorie se foloseste utilitarul _avrdude_.

`avrdude -c arduino -P /dev/ttyUSB0 -b 57600 -p atmega324p -U flash:w:lab0.hex:a`


**Legatura cu mediul extern**

_actuatorii_ = elemente care influenteaza mediul exterior
_traductoarele_ = elemente care isi modifica proprietatile electrice in functie de parametrii mediului exterior
_LED_(Light Emitting Diode) = numite și diode electroluminesciente emit lumină când ele sunt polarizate direct
_butoane_ = se foloseste o rezistenta de pull-up/pull-down pentru a evita starea de impedanta marita (stare nedefinita, pinul este "in aer")

Rezistentele de pull-up/pull-down sunt incluse in microcontroller --> cand se scrie in PORTn aceastea sunt activate.

**GPIO**
_DDRn_ (Data Direction Register)
* stabilește direcţia pinilor portului
* dacă bitul x are valoarea 0 atunci pinul x este de intrare
* dacă bitul x are valoarea 1 atunci pinul x este de ieșire

_PORTn_ (Data Register)
* stabilește valorile de ieşire ale pinilor sau activează/dezactivează rezistenţele de pull-up
* dacă bitul x are valoarea 0 atunci
	> dacă pinul x este de ieșire el va avea valoarea LOW
	> daca pinul x este de intrare rezistența de pull-up va fi dezactivată
* daca bitul x are valoarea 1 atunci
 	> daca pinul x este de ieșire el va avea valoarea HIGH
	> dacă pinul x este de intrare rezistența de pull-up va fi activată

_PINn_ (Input Pins Address)
* putem citi date de pe portul respectiv
* dacă pinul x are valoarea LOW atunci bitul x va avea valoarea 0
* daca pinul x are valoarea HIGH atunci bitul x va avea valoarea 1

Registrele DDRn, PORTn, PINn, precum şi registrele perifericelor nu fac parte din cele 32 de registre de uz general. Ele nu sunt folosite pentru stocarea datelor, ci au ca rol comunicarea cu perifericul corespunzător. Pentru a face accesul la acestea mai simplu, ele sunt mapate în memorie. Astfel, pentru a scrie o valoare într-un registru este necesar să se scrie valoarea la adresa de memorie corespunzătoare registrului. Adresele lor se pot vedea în datasheet, în capitolul Register Summary, iar în avr-libc sunt definite macro-uri pentru fiecare dintre aceste adrese.


|Operatie                       		|Forma                  	   	 |
|---------------------------------------|--------------------------------|
|Scriere bit pe 1              		    |`register \|= (1 << bit_index)` |
|Scriere bit pe 0               		|`register &= ~(1 << bit_index)` |
|Toggle bit                             |`register ^= (1 << bit_index)`  |
|Citire bit								|`register & (1 << bit_index)`	 |

**Exemplu de lucru cu ieșiri**
Să presupunem că avem un LED legat la pinul 1 al portului B (numit PORTB1 sau PB1). Pentru a aprinde sau stinge LED-ul trebuie să urmăm următorii pași:

* pinul PB1 trebuie configurat ca ieșire - bitul 1 (PB1) din registrul DDRB va fi 1
* pentru a aprinde LED-ul trebuie ca pinul PB1 sa ia valoarea HIGH- bitul 1 (PB1) din registrul PORTB va fi 1
* pentru a stinge LED-ul trebuie ca pinul PB1 sa ia valoarea LOW - bitul 1 (PB1) din registrul PORTB va fi 0

**Exemplu de lucru cu intrări**
Să presupunem ca avem un buton legat la pinul 4 al portului D (numit PORTD4 sau PD4), din cazul b) prezentat anterior. Pentru a determina starea butonului (apăsat sau liber) trebuie să urmăm următorii pași:

* pinul PD4 trebuie configurat ca intrare - bitul 4 (PD4) din registrul DDRD va fi 0
* pentru a determina starea de apăsare a butonului trebuie să citim valoarea pinului la care este atașat. Acesta va fi 1 atunci când butonul este liber și 0 atunci când butonul este apăsat
* citim valoarea bitului 4 (PD4) din registrul PIND

Exemplu program care stinge și aprinde un LED la intervale de 500 ms:

```
// lab0.c

#include <avr/io.h>
#include <util/delay.h>
 
int main() {
	/* Setăm pinul 0 al portului C ca pin de ieșire. */
	DDRC |= (1 << PC0);
 
	while(1) {
                /* Inversăm starea pinului. */
		PORTC ^= (1 << PC0);
 
                _delay_ms(500);
	}
 
	return 0;
}
```

Exemplu Makefile:
```
# Linux
PORT ?= /dev/ttyUSB0
# Windows
# PORT ?= COM1
 
all: lab0.hex
 
lab0.hex: lab0.elf
	avr-objcopy  -j .text -j .data -O ihex $^ $@
	avr-size lab0.elf
 
lab0.elf: lab0.c
	avr-g++ -mmcu=atmega324p -DF_CPU=12000000 -Os -Wall -o $@ $^
 
upload: lab0.hex
	avrdude -c arduino -P $(PORT) -b 57600 -p atmega324p -U flash:w:$<:a
 
clean:
	rm -rf lab0.elf lab0.hex
```
Compilatorul folosit este **avr-gcc**. Flag-urile au următoarea semnificație:
* _mmcu_: Informează compilatorul despre tipul microcontroller-ului pentru care trebuie să genereze codul
* _DF_CPU=12000000_: Definește macro-ul F_CPU care indică frecvența de lucru la microcontroller-ului
* _Os_: Optimizează programul în privința memoriei ocupate

Fișierul care poate fi programat pe plăcuță trebuie să fie în formatul ihex, de aceea este necesară extragerea secțiunilor de date și cod din fișierul elf obținut în urma compilării integrarea lor într-un fișier ihex. Acest pas folosește utilitarul *avr-objcopy*.

Comanda avr-size arată câtă memorie ocupă diverse secțiuni de cod ale programului. Trebuie avut în vedere că în memoria SRAM de date, în afară de secțiunea .data, o să fie pusă și stiva. Pentru a putea rula programul pe microcontroller-ul ATmega324 trebuie că secțiunea .text să fie <32 KB și secțiunea .data să fie <2 KB. Rezultatul compilării este fișierul lab0.hex. Acesta urmează a fi încărcat în microcontroller folosind regula upload.