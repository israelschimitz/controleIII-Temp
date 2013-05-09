
// Sistema de controle fuzzy de processo não linear.
// Controle II
// Israel - Piccoli - Luis Henrique

#include "pins_arduino.h"
#include "avr/io.h"
#include "stdint.h"
#include "avr/interrupt.h"
#include "LiquidCrystal.h"

// set display
LiquidCrystal lcd( 8, 9, 4, 5, 6, 7 );

// Variaveis globais
float gd = 8;           /// Ganho do controlador fuzzy.
float ref = 0;          /// Referencia de temperatura;
float ea = 0;           /// Erro anterior
float dt = 1;           /// Incremento de tempo.
float Iy = 0;           /// Sinal de comando do fuzzy;
float integral = 0;     /// Parcela integral do controlador.
int data[2];            /// Vetor de dados recebidos da serial
int ligado = 0;         ///


/// Declaração das funções ///
float fcfuzpdp(signed int e, signed int v);
int fuzzificacao (signed int centro, signed int base,signed int x);
void establishContact();
int transformar(int leituraa);

/// Pino de comando do relé de estado sólido ///
const int saida = 51;

/// Setup ///
void setup() {
  
  //Serial.begin(9600);
  pinMode(saida, OUTPUT);
  //establishContact();
  
  //LCD
  lcd.begin(16, 2);             // LCD - configura o número de colunas e linhas 
  lcd.clear();                  // limpa display LCD

  delay(200);
}

/// Configuração da serial e aguarda comunicação ///
void establishContact() {
    while(Serial.available() <= 0) {
        Serial.println('A');
        delay(50);
    }
    while(Serial.available()>0){
        int q = Serial.read();
    }
}

/// Loop ///
void loop() {
/// -------------descomentar---------
/*
    if(Serial.available() == 2){
        data[0] = Serial.read();
        ligado = data[0];
        data[1] = Serial.read();
        ref = (float)data[1];
    }
*/
/// -------------descomentar---------

///-------------apagar--------------
  ref=80;
  ligado=1; 
///-------------apagar--------------

     
    if(ligado > 0){

        int temperatura = analogRead(A8);   /// Leitura da temperatura e conversão para ºC ///
        temperatura = map(temperatura,0,1023,0,500);

        float err = ref - temperatura;      /// Erro.
        float err1 = 1 / ref;
        float  der = (err - ea);            /// Variação do erro.
        float der1 = 1 / ref;
        ea = err;

        float Erro = err1 * err;            /// Normalizado valor do erro.
        if(Erro > 1) {
            Erro = 1;
        }
        if(Erro < -1){
            Erro = -1;
        }

        float Verr = der1 * der;            /// Normalizado valor da variação do erro.
        if(Verr > 1){
            Verr = 1;
        }
        if(Verr < -1){
            Verr = -1;
        }

        Erro = Erro * 100;                  /// Ajuste de escalas
        Verr = Verr * 100;
        signed int Erro1 = (int)Erro;
        signed int Verr1 = (int)Verr;

        float Defuz = fcfuzpdp(Erro1,Verr1);    /// Função do controlador fuzzy.

        integral += err;              /// err = err * dt, fator integral do controlador
        if(integral > 50){              /// Limite de saturação do integrador
            integral = 50;
        }

        Iy = (Defuz * gd) + integral;   /// Sinal de controle do controlador, Proporcional + integral
        if(Iy > 800){
            Iy = 800;                   /// Limite de saturação do controlador
        }
        if(Iy < 0){
            Iy = 0;
        }

        signed int potencia = (int)Iy;
        int pot_positiva = 0;
        unsigned int potencia1 = 0;
        if( potencia >= 0){
            pot_positiva = potencia;
            potencia1 = map(potencia,0,800,0,1000); /// Ajuste entre a potência do atuador
        }else {                                     /// e o tempo que permanecerá ligado
            potencia1 = 0;
            pot_positiva = 0;
            }

        unsigned long agora = millis();
        unsigned long tempo = agora + potencia1;
        unsigned long agora1 = agora;
        while(agora1 < tempo){
            agora1 = millis();
            digitalWrite(saida, HIGH);
        }
        unsigned long agora2 = agora + 1000;
        while(agora2 >= agora1){
            agora1 = millis();
            digitalWrite(saida, LOW);
        }

        long valor = 0;
        valor = (long)pot_positiva;
       
        lcd.clear();
        lcd.setCursor(0,0);         //Posição do cursor
        lcd.print(Iy);
         
        lcd.setCursor(0,1);         //Posição do cursor
        lcd.print(temperatura);
        
        //delay(1000);
        
        
    /*    valor = valor<<8;               /// Valor de temperatura e potencia
        valor += temperatura;           /// São unidos numa única variável.
        Serial.println(valor);          /// Envio de dados para o Matlab.
      */  
    }

}

float fcfuzpdp(signed int e, signed int v){

    ///Definicao das funcoes de pertinencia (tipo triangulares).
    signed int ceNM = -10;
    signed int ceZE = 0;
    signed int cePP = 10;
    signed int cePM = 20;
    signed int cePG = 30;

    signed int beNM = 11;
    signed int beZE = 10;
    signed int bePP = 9;
    signed int bePM = 10;
    signed int bePG = 10;

    signed int cvNM = -30;
    signed int cvZE = 0;
    signed int cvPP = 30;
    signed int cvPM = 60;
    signed int cvPG = 100;

    signed int bvNM = 30;
    signed int bvZE = 30;
    signed int bvPP = 30;
    signed int bvPM = 30;
    signed int bvPG = 40;

    signed int caNM = -30;
    signed int caZE = 0;
    signed int caPP = 30;
    signed int caPM = 60;
    signed int caPG = 100;

    signed int ueNM = 0;
    signed int ueZE = 0;
    signed int uePP = 0;
    signed int uePM = 0;
    signed int uePG = 0;

    signed int uvNM = 0;
    signed int uvZE = 0;
    signed int uvPP = 0;
    signed int uvPM = 0;
    signed int uvPG = 0;


    /// Fuzzyficação;
    if(e < ceNM) {
        ueNM = -100;
    }else {
        ueNM = fuzzificacao (ceNM, beNM,e);
        }
    ueZE = fuzzificacao (ceZE, beZE,e);
    uePP = fuzzificacao (cePP, bePP,e);
    uePM = fuzzificacao (cePM, bePM,e);
    if (e > cePG){
        uePG = 100;
    }else uePG = fuzzificacao (cePG, bePG,e);


    if(v < cvNM){
        ueNM = -100;
    }else{
        uvNM = fuzzificacao (cvNM, bvNM,v);
    }
    uvZE = fuzzificacao (cvZE, bvZE,v);
    uvPP = fuzzificacao (cvPP, bvPP,v);
    uvPM = fuzzificacao (cvPM, bvPM,v);
    if (v > cvPG){
        uvPG = 100;
    }else{
        uvPG = fuzzificacao (cvPG, bvPG,v);
    }

    /// Agregacao
    signed long r1 = ueNM * uvNM;
    signed long r2 = ueNM * uvZE;
    signed long r3 = ueNM * uvPP;
    signed long r4 = ueNM * uvPM;
    signed long r5 = ueNM * uvPG;
    signed long r6 = ueZE * uvNM;
    signed long r7 = ueZE * uvZE;
    signed long r8 = ueZE * uvPP;
    signed long r9 = ueZE * uvPM;
    signed long r10 = ueZE * uvPG;
    signed long r11 = uePP * uvNM;
    signed long r12 = uePP * uvZE;
    signed long r13 = uePP * uvPP;
    signed long r14 = uePP * uvPM;
    signed long r15 = uePP * uvPG;
    signed long r16 = uePM * uvNM;
    signed long r17 = uePM * uvZE;
    signed long r18 = uePM * uvPP;
    signed long r19 = uePM * uvPM;
    signed long r20 = uePM * uvPG;
    signed long r21 = uePG * uvNM;
    signed long r22 = uePG * uvZE;//d
    signed long r23 = uePG * uvPP;
    signed long r24 = uePG * uvPM;
    signed long r25 = uePG * uvPG;

    /// Defuzzificação
    signed long d = 0;
    d += caNM * r1;
    d += caNM * r2;
    d += caZE * r3;
    d += caPP * r4;
    d += caPM * r5;
    d += caNM * r6;
    d += caZE * r7;
    d += caPP * r8;
    d += caPM * r9;
    d += caPM * r10;
    d += caZE * r11;
    d += caPP * r12;
    d += caPM * r13;
    d += caPM * r14;
    d += caPG * r15;
    d += caPP * r16;//;
    d += caPM * r17;
    d += caPM * r18;
    d += caPG * r19;//;
    d += caPG * r20;//;
    d += caPM * r21;
    d += caPG * r22;
    d += caPG * r23;
    d += caPG * r24;
    d += caPG * r25;

    float dd = d / (r1+r2+r3+r4+r5+r6+r7+r8+r9+r10+
                    r11+r12+r13+r14+r15+r16+r17+r18+
                    r19+r20+r21+r22+r23+r24+r25);
    return(dd);

}

int fuzzificacao (signed int centro, signed int base,signed int x)
{
    unsigned int resultado = 0;
    signed int calc1 = centro - base;
    signed int calc2 = centro + base;

    if ((calc1 <= x) && (x < calc2)){
         signed int aux = abs(centro - x);
         aux =  aux * 100;
         signed  int aux1 = aux / base;
         resultado = 100 - aux1;
    }
    return (resultado);
}
