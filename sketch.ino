// Realizzare, tramite un Arduino NANO, un sistema di irrigazione intelligente 
// con le seguenti caratteristiche:
// 1) sensore di umidità, simulato in wokwi da un potenziometro.
// 2) Rilevazione temperatura e umidità ambiente tramite DHT2.
// 3) Relay per comando elettrovalvola irrigazione.
// Il programma deve poter consentire di impostare:
// a) Tramite interfaccia seriale, al reset, la data ed ora corrente e fino 
//     a 16 orari, all'interno di ciascuna giornate per avvio di un 
//     task di controllo ed eventuale irrigazione, specificandone la durata 
// b) I livelli massimi (troppo umido) e minimi (troppo secco) previsti e quindi decidere,
//    in ciascun ciclo di controllo, se avviare o meno il relay di irrigazione.
// c) dei parametri correttivi sulla durata di irrigazione, 
//    in base alla temperatura ed umidità ambiente
// Il sistema si pone in stato di “Power Down”, una volta terminato un ciclo 
// di attività e potrà essere sveglialo da:
// 1) dalla premuta di un pulsante per avvio immediato di un ciclo di controllo. 
// 2) dall’orario di avvio di un ciclo di controllo.
// La gestione del DHT22 e dell'ingresso analogico dovrà essere svolta 
// senza usare le librerie specifiche.


// Definisco bit e pin componenti:

// DHT22
#define PIN_DHT22 3
#define BIT_DHT22 (1 << PIN_DHT22)

// SLIDE POTENTIOMETER
#define PIN_POT 0

// RELAY MODULE
#define PIN_RELAY 2
#define BIT_RELAY (1 << PIN_RELAY)

// PUSHBUTTON
#define PIN_PB 6
#define BIT_PB (1 << PIN_PB)

// Definizione dei livelli Minimo e Massimo di umidità del suolo
#define MIN_HUMIDITY 30  
#define MAX_HUMIDITY 70 

// Definizione degli intervalli in secondi tra le letture dei sensori e l'intervallo giornaliero
#define INTERVAL_DHT22 7200
#define INTERVAL_POT 3600
#define INTERVAL_DAY 86400

// Funzione personalizzata per impostare il livello di un pin digitale
#define setDigitalLevel(pin, level) (level == HIGH ? (PORTD |= pin) : (PORTD &= ~pin))

// Funzione personalizzata per impostare la modalità di un pin
#define setPinMode(pin, mode) (mode == OUTPUT ? (DDRD |= pin) : (DDRD &= ~pin)); \ 
(mode == INPUT_PULLUP ? (PORTD |= (1 << pin)) : (NULL))

// Funzione personalizzata per leggere un pin digitale
bool readDigitalPin(uint8_t pin) {
  if (PIND & (1 << pin))
    return true;
  else
    return false;
}

// Contatori per gli intervalli tra le letture dei sensori e il contatore giornaliero
volatile uint32_t counterDay = INTERVAL_DAY;
volatile uint32_t pos = 0;
volatile uint16_t counterSecond = 0;

// Variabili globali per memorizzare i valori delle misurazioni
float temperature, humidity, soilHumidity;
// Inizializzo Ora e Minuti correnti, il Contatore degli orari di irrigazione impostati,
// gli Array degli orari di irrigazione e gli Array delle durate di irrigazione
uint8_t currentHour, currentMinute, irrigationCount;
uint32_t irrigationTimes[16], irrigationDurations[16];
uint32_t iTimes[16], iDurations[16];


void setup() {
  Serial.begin(9600); // Avvio la comunicazione seriale
  TCCR1A = 0; // Azzero il registro di controllo A del timer 1
  OCR1A = 15624; // Registro di confronto per l'interrupt ogni secondo
  TCCR1B |= (1 << WGM12); // Abilito la modalità di confronto CTC
  TCCR1B = (TCCR1B & 0xF8) | 0x05; // Imposto il prescaler a 1024
  TIMSK1 |= (1 << OCIE1A); // Abilito l'interrupt di confronto A del timer 1
  PCICR |= (1 << PCIE2); // Attivo il PIN change Interrupt per i PIN dal 23 al 16
  PCMSK2 |= (1 << PCINT22); // Attivo il PIN change Interrupt solo per il PIN 22
  setPinMode(PD6, INPUT_PULLUP); // Imposto il PushButton come INT_PULLUP
  SerialSetUp();
  Serial.println("Orari di controllo aggiornati con successo!");
}

void loop() {
}


// Funzione per leggere il sensore DHT22
void readDHT22(uint8_t pin) {
  uint8_t data[5] = {0, 0, 0, 0, 0};
  // Segnale di avvio
  setPinMode(BIT_DHT22, OUTPUT);
  setDigitalLevel(BIT_DHT22, LOW);
  delayMicroseconds(1000);
  setDigitalLevel(BIT_DHT22, HIGH);
  delayMicroseconds(40);
  // Segnale di risposta
  setPinMode(BIT_DHT22, INPUT);
  while (readDigitalPin(BIT_DHT22));
  while (!readDigitalPin(BIT_DHT22));
  while (readDigitalPin(BIT_DHT22));
  // Lettura dei 40 bit disabilitando gli interrupt
  cli();
  for (uint8_t i = 0; i < 5; i++) {
    for (uint8_t j = 0; j < 8; j++) {
      while (!readDigitalPin(BIT_DHT22));
      delayMicroseconds(50);
      data[i] = (data[i] << 1);
      if (readDigitalPin(BIT_DHT22))
        data[i] |= 0x01;
      while (readDigitalPin(BIT_DHT22));
    }
  }
  sei();
  // Verifica del checksum e salvataggio dei valori nelle variabili globali
  if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
    humidity = ((data[0] << 8) | data[1]) / 10.0;
    temperature = ((data[2] & 0x7F) << 8 | data[3]) / 10.0;
    if (data[2] & 0x80) {
      temperature = -temperature;
    }
  }
}


// Funzione per leggere il potenziometro
void readPotentiometer(uint8_t pin) {
  ADMUX = (pin & 0xF8) | (pin & 0x07);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  soilHumidity = ADC * (100.0 / 1023.0);
}


// Funzione per richiamare l'Interrupt per il TIMER1 sulla comparazione con OCR1A
ISR(TIMER1_COMPA_vect) {
  if (--irrigationTimes[pos] == 0) {
    irrigationTimes[pos] = iTimes[pos];
    irrigationTimes[pos+1] -= irrigationTimes[pos];
    pos += 1;
    readDHT22(PIN_DHT22); // Leggo il DHT22
    readPotentiometer(PIN_POT); // Leggo il Potenziometro
    if (soilHumidity < MIN_HUMIDITY) { // Controllo il livello di umidità
      setDigitalLevel(BIT_RELAY, HIGH); // Attiva il relay per l'irrigazione per la durata necessaria
    }
  }
  if (--counterDay == 0) {
    counterDay = INTERVAL_DAY;
    pos = 0;
  }
}


// Funzione per gestire la pressione del PushButton
ISR(PCINT2_vect){
  if (readDigitalPin(PD6) == false && counterSecond > 0) {
    Serial.println("Pulsante premuto!");
    Serial.println("Avvio Ciclo di Controllo...");
    counterSecond = 0;
  }
}

// Funzione per gestire la comunicazione Seriale
void SerialSetUp(){
  Serial.println("Programma una Schedule Orari di Controllo :");
  Serial.println("(Inserire un numero > 24 per terminare)");
  for(uint32_t i = 0; i < 16; i++){
    while (Serial.available() == 0);  // Se la comunicazione Seriale non è disponibile aspetta
    irrigationTimes[i] = Serial.parseInt();
    if (irrigationTimes[i] <= 24) {
      irrigationCount = i;
      iTimes[i] = irrigationTimes[i];
      Serial.println("Orario inserito: ");
      Serial.println(irrigationTimes[i]);
      while (Serial.available() == 1);  // Aspetto che la linea seriale sia nuovamente bassa 
    }
    else{
      break;
    }
  }
}
