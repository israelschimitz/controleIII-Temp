
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
float GanhoControlador = 20;           /// Ganho do controlador fuzzy.
float Referencia = 0;          /// Referencia de temperatura;
float ErroAnterior = 0;           /// Erro anterior
float IncrementoTempo = 1;               /// Incremento de tempo.
float SaidaAtuador = 0;           /// Sinal de comando do fuzzy;
float Integrador = 0;     /// Parcela integral do controlador.
int   RecebidosSerial[2];            /// Vetor de dados recebidos da serial
int   BotaoLigado = 0;         ///


/// Declaração das funções ///
float Fuzzy(signed int e, signed int v);
int Fuzzificacao (signed int centro, signed int base,signed int x);
void EstabeleceContato();

/// Pino de comando do relé de estado sólido ///
int PinoSaida = 3;

/// Setup ///
void setup() {
  
  Serial.begin(9600);
  pinMode(PinoSaida, OUTPUT);
  EstabeleceContato();
  
//LCD
  lcd.begin(16, 2);             // LCD - configura o número de colunas e linhas 
  lcd.clear();                  // limpa display LCD

  delay(200);
}

/// Configuração da serial e aguarda comunicação ///
void EstabeleceContato() {
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

    if(Serial.available() == 2){
        RecebidosSerial[0] = Serial.read();
        BotaoLigado = RecebidosSerial[0];
        RecebidosSerial[1] = Serial.read();
        Referencia = (float)RecebidosSerial[1];
    }

    if(BotaoLigado > 0){

        int LeituraTemperatura = analogRead(A8);   /// Leitura da temperatura e conversão para ºC ///
        
        if(LeituraTemperatura > 245)
          LeituraTemperatura = 245;

        LeituraTemperatura = map(LeituraTemperatura,0,1023,0,500);
        
        float err = Referencia - LeituraTemperatura;      /// Erro.
        float err1 = 1 / Referencia;
        float  der = (err - ErroAnterior)/IncrementoTempo;            /// Variação do erro.
        float der1 = 1 / Referencia;
        ErroAnterior = err;

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

        float Defuz = Fuzzy(Erro1,Verr1);    /// Função do controlador fuzzy.

        Integrador += err;              /// err = err * dt, fator integral do controlador
        if(Integrador > 50){              /// Limite de saturação do integrador
            Integrador = 50;
        }

        SaidaAtuador = (Defuz * GanhoControlador) + Integrador;   /// Sinal de controle do controlador, Proporcional + integral
        if(SaidaAtuador > 2000){
            SaidaAtuador = 2000;                   /// Limite de saturação do controlador
        }
        if(SaidaAtuador < 0){
            SaidaAtuador = 0;
        }

        signed int potencia = (int)SaidaAtuador;
        int pot_positiva = 0;
        int potencia1 = 0;
        if( potencia >= 0){
            pot_positiva = potencia;
            potencia1 = map(potencia,0,2000,0,255); /// Ajuste entre a potência do atuador
        }else {                                     /// e o tempo que permanecerá ligado
            potencia1 = 0;
            pot_positiva = 0;
            }

   // PWM
        analogWrite(PinoSaida, potencia1);   
       
   //atualiza o LCD 
        lcd.clear();
        lcd.setCursor(0,0);         //Posição do cursor
        lcd.print("P:");
        lcd.setCursor(2,0);         //Posição do cursor
        lcd.print(SaidaAtuador);
         
        lcd.setCursor(10,0);         //Posição do cursor
        lcd.print("T:");
        lcd.setCursor(13,0);  
        lcd.print(LeituraTemperatura);
        
        lcd.setCursor(0,1);  
        lcd.print("E:");
        lcd.setCursor(2,1);  
        lcd.print(Erro);
        lcd.setCursor(8,1);  
        lcd.print("V:");
        lcd.setCursor(11,1);  
        lcd.print(Verr);
     
        delay(1000);
                  
        // atualiza a serial           
        long valor = 0;
        valor = (long)pot_positiva/10;
        valor = valor<<8;               /// Valor de temperatura e potencia
        valor += LeituraTemperatura;           /// São unidos numa única variável.
        Serial.println(valor);          /// Envio de dados para o Matlab.
        
    }

}

float Fuzzy(signed int e, signed int v){

    ///Definicao das funcoes de pertinencia (tipo triangulares).
    signed int ceNM = -10;  //CENTRO DO ERRO
    signed int ceNS = -5;
    signed int ceZE = 0;
    signed int cePS = 5;
    signed int cePM = 10;

    signed int beNM = 5; // BASE DO ERRO
    signed int beNS = 5; 
    signed int beZE = 1;
    signed int bePS = 5;
    signed int bePM = 5;

    signed int cvNM = -10; 
    signed int cvNS = -5;//CENTRO DO VERR
    signed int cvZE = 0; 
    signed int cvPS = 5;
    signed int cvPM = 10;

    signed int bvNM = 5;  // BASE DO VERR
    signed int bvNS = 5;
    signed int bvZE = 1;  
    signed int bvPS = 5;
    signed int bvPM = 5;

    signed int caNM = -50;
    signed int caNS = -25;
    signed int caZE = 0;
    signed int caPS = 30;
    signed int caPM = 100;

    signed int ueNM = 0;
    signed int ueNS = 0;
    signed int ueZE = 0;
    signed int uePS = 0;
    signed int uePM = 0;

    signed int uvNM = 0;
    signed int uvNS = 0;
    signed int uvZE = 0;
    signed int uvPS = 0;
    signed int uvPM = 0;


    /// Fuzzyficação;
    if(e < ceNM) {
        ueNM = -100;
    }else {
        ueNM = Fuzzificacao (ceNM, beNM,e);
        }
    ueNS = Fuzzificacao (ceNS, beNS,e);
    ueZE = Fuzzificacao (ceZE, beZE,e);
    uePS = Fuzzificacao (cePS, bePS,e);
    if (e > cePM){
        uePM = 100;
    }else uePM = Fuzzificacao (cePM, bePM,e);


    if(v < cvNM){
        ueNM = -100;
    }else{
        uvNM = Fuzzificacao (cvNM, bvNM,v);
    }
    uvNS = Fuzzificacao (cvNS, bvNS,v);
    uvZE = Fuzzificacao (cvZE, bvZE,v);
    uvPS = Fuzzificacao (cvPS, bvPS,v);
    if (v > cvPM){
        uvPM = 100;
    }else{
        uvPM = Fuzzificacao (cvPM, bvPM,v);
    }

    /// Agregacao
    signed long r1 = ueNM * uvNM;
    signed long r2 = ueNM * uvNS;
    signed long r3 = ueNM * uvZE;
    signed long r4 = ueNM * uvPS;
    signed long r5 = ueNM * uvPM;
    signed long r6 = ueNS * uvNM;
    signed long r7 = ueNS * uvNS;
    signed long r8 = ueNS * uvZE;
    signed long r9 = ueNS * uvPS;
    signed long r10 = ueNS * uvPM;
    signed long r11 = ueZE * uvNM;
    signed long r12 = ueZE * uvNS;
    signed long r13 = ueZE * uvZE;
    signed long r14 = ueZE * uvPS;
    signed long r15 = ueZE * uvPM;
    signed long r16 = uePS * uvNM;
    signed long r17 = uePS * uvNS;
    signed long r18 = uePS * uvZE;
    signed long r19 = uePS * uvPS;
    signed long r20 = uePS * uvPM;
    signed long r21 = uePM * uvNM;
    signed long r22 = uePM * uvNS;
    signed long r23 = uePM * uvZE;
    signed long r24 = uePM * uvPS;
    signed long r25 = uePM * uvPM;

    /// Defuzzificação
    signed long d = 0;
    d += caNM * r1;
    d += caNS * r2;
    d += caZE * r3;
    d += caZE * r4;
    d += caNS * r5;
    d += caNS * r6;
    d += caNS * r7;
    d += caNS * r8;
    d += caZE * r9;
    d += caNS * r10;
    d += caZE * r11;
    d += caZE * r12;
    d += caZE * r13;
    d += caZE * r14;
    d += caZE * r15;
    d += caPS * r16;
    d += caPS * r17;
    d += caZE * r18;
    d += caPS * r19;
    d += caPM * r20;
    d += caPM * r21;
    d += caPM * r22;
    d += caPM * r23;
    d += caPM * r24;
    d += caPM * r25;

    float dd = d / (r1+r2+r3+r4+r5+r6+r7+r8+r9+r10+
                    r11+r12+r13+r14+r15+r16+r17+r18+
                    r19+r20+r21+r22+r23+r24+r25);
    return(dd);

}

int Fuzzificacao (signed int centro, signed int base,signed int x)
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
