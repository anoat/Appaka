#include <math.h>
#include <SPI.h>
#include <Pixi.h>

Pixi pixi;

const byte VCF_VCA = 2; // Enveloppe for VCF and VCA
const byte ADSR = 4; // pot control
const byte SW_GTF = 3; // switch control

const byte MIDI_L[VCF_VCA]    = {  115,    116};
const byte HR_MIDI[VCF_VCA][ADSR] = {{ 44, 45, 46, 47}, { 48, 49, 50, 51}};
const byte MIDI_GTF [SW_GTF][VCF_VCA] = {{ 103,    108}, { 104,    109}, { 105,    110}};

float         k_timer[VCF_VCA]         = {0, 0};  //

unsigned long TimerStartTop[VCF_VCA]   = {0, 0};  // instant en microsecondes du dernier evenement start ou stop
unsigned long TimeRefA[VCF_VCA]        = {0, 0};  // Duree en microsecondes de l'Attack
unsigned long TimeRefD[VCF_VCA]        = {0, 0};  // Duree en microsecondes du Decay
unsigned long TimeRefR[VCF_VCA]        = {0, 0};  // Duree en microsecondes du Release
unsigned long tempsreel[VCF_VCA];                 // Differentiel du Temps en microseconde entre l'evenement et le temps absolu
unsigned long  Offset[VCF_VCA]         = {0, 0};  // Amortissement des montees et descente du MCP

int POT[VCF_VCA][ADSR];
int AMT[VCF_VCA];
int LEVEL [VCF_VCA];
int OUT[VCF_VCA];
int out_value;
int  tension[VCF_VCA]                   = {0, 0};  // Tension courante en millivolts
int  tensionlin[VCF_VCA]                = {0, 0};  // Tension courante en millivolts
int  tensionlog[VCF_VCA]                = {0, 0};  // Tension courante en millivolts
int  memo_tension[VCF_VCA]              = {0, 0};  // Tension boucle-1 en millivolts
unsigned int  TensionRefS[VCF_VCA]      = {0, 0};  // Tension (mV) de reference du SUSTAIN
unsigned int  TensionStartTop[VCF_VCA]  = {0, 0};  // Tension (mV) du dernier evenement start ou stop

byte          klog                     = 4;       // Courbure equivalente à celles des condensateurs
byte          Etat_module[VCF_VCA]     = {0, 0};  // 0=NOP      1=A       2=D     3=S     4=R
byte          Choix_forme[VCF_VCA]     = {0, 0};  // 0=LIN      1=LOG     2=EXP
byte          Choix_timer[VCF_VCA]     = {0, 0};  // 0=x1       1=x0.1    2=x10
byte          Choix_gate_in[VCF_VCA]   = {0, 0};  // 0=GATE_IN  1=TRIG_IN 2=LOOP
byte          DureeRef[3] = {5, 10, 60};
byte          high_byte[17][33];   // High resolution MIDI
byte          Etat_SW [VCF_VCA][SW_GTF];

boolean       Etat_gate_in    = 0;   // Copie de l'etat de l'entree gate  (1=on  0=off)
boolean       StartTop        = 0;   // Flag fugitif sur l'Etat_gate_in  (1=actif  0=deactive)


void setup() {

  usbMIDI.setHandleNoteOn(OnNoteOn);
  usbMIDI.setHandleNoteOff(OnNoteOff);
  usbMIDI.setHandleControlChange(OnControlChange);

  word Pixi_ID = 0;
  float Temp = 0;
  word test = 0;

  Pixi_ID = pixi.config();

  // Identify shield by ID register
  if (Pixi_ID == 0x0424 ) {

    pixi.configChannel ( CHANNEL_10, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannel ( CHANNEL_11, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );         // MODE 5 = DAC OUTPUT
  }
}

void loop() {

  usbMIDI.read();
  AUTOMATE();           // GESTION DE L'ENCHAINEMENT DES TACHES
  CALCUL();             // DETERMINATION DE LA TENSION DE SORTIE
  ECRIREDAC();          // ENVOIE DE LA TENSION DE SORTIE SUR LE DAC
  IHM();                // GESTION DES INVERSEURS ET POTENTIOMETRES
}

//----------------------------------------------
// Gestion du Gate_In par MIDI
//----------------------------------------------

void OnNoteOn(byte channel, byte pitch, byte velocity) {
  if (velocity > 0) {
    Etat_gate_in = 1;
    StartTop = 1;
    AMT[0] = map(velocity, 0, 127, 0, LEVEL[0]);
    AMT[1] = map(velocity, 0, 127, 0, LEVEL[1]);
  }
  else {
    Etat_gate_in = 0;
  }
}

void OnNoteOff(byte channel, byte pitch, byte velocity) {
  Etat_gate_in = 0;
}
//----------------------------------------------
// ENVOI DE LA TENSION DE SORTIE AU DAC
//----------------------------------------------

void ECRIREDAC() {
  for (int i = 0; i < VCF_VCA ; i++) {
    if (memo_tension[i] != tension[i]) {
      memo_tension[i] = tension[i];

      OUT[i] = map(tension[i], 0, 4095, 0, AMT[i]);

      pixi.writeAnalog ( CHANNEL_10, OUT[0]);
      pixi.writeAnalog ( CHANNEL_11, OUT[1]);
    }
  }
}

//----------------------------------------------
// CALCUL DE LA TENSION DE SORTIE (0->4095mV)
//----------------------------------------------
void CALCUL() {
  for (int i = 0; i < VCF_VCA ; i++)
    //----------------------------------------------------------------------ATTACK--
    if (Etat_module[i] == 1) {
      tempsreel[i] = (micros() - TimerStartTop[i]);
      tension[i] = 4095 - TensionStartTop[i];
      tension[i] *= float(tempsreel[i] * k_timer[i] / (TimeRefA[i] * (4095.00 - TensionStartTop[i]) / 4095.00));
      tension[i] += TensionStartTop[i];
      if      (Choix_forme[i] == 1) {
        tension[i] += 512;
        float k = (4095 - TensionStartTop[i]) / 3.61;
        tension[i] = (log10(tension[i] - TensionStartTop[i]) * k * klog) + (4095 - (log10(4095 - TensionStartTop[i]) * k * klog));
      }
      if      (Choix_forme[i] == 2) {
        tensionlin[i] = tension[i];
        tension[i] = 4095 - TensionStartTop[i];
        float k = tension[i] / 3.61;
        tension[i] *= float((TimeRefA[i] - (tempsreel[i] * k_timer[i])) / (TimeRefA[i] * (4095.00 - TensionStartTop[i]) / 4095.00));
        tensionlog[i] = (log10(tension[i] - TensionStartTop[i]) * k * klog) + (4095 - (log10(4095 - TensionStartTop[i]) * k * klog));
        tension[i] = tensionlin[i] - tensionlog[i] + tension[i];

      }
      tension[i] = constrain(tension[i], TensionStartTop[i], 4095);
    }

  //----------------------------------------------------------------------DECAY--
    else if (Etat_module[i] == 2) {
      tempsreel[i] = (micros() - TimerStartTop[i]);
      tension[i] = TensionStartTop[i] - ((TensionStartTop[i] - TensionRefS[i]) * (k_timer[i] * tempsreel[i] / TimeRefD[i] * 2));
      if      (Choix_forme[i] == 1) { //LOG => EXP en descente
        float k = log(TensionStartTop[i] - TensionRefS[i]) * (1 - (k_timer[i] * tempsreel[i] / TimeRefD[i]));
        if (k > 16) {
          tension[i] = 4095;
        } else {
          tension[i] = exp(k) + TensionRefS[i] - 1;
        }
      }
      else if (Choix_forme[i] == 2) { // EXP => LOG en descente

        tension[i] = TensionRefS[i] + ((log10((TensionStartTop[i] - (TensionStartTop[i] * (k_timer[i] * tempsreel[i] / TimeRefD[i] * 2))) / (klog * 100))) * ((TensionStartTop[i] - TensionRefS[i]) / log10(TensionStartTop[i] / (klog * 100))));
      }
      tension[i] = constrain(tension[i], TensionRefS[i] - 1, 4095);
    }

  //----------------------------------------------------------------------SUSTAIN--
    else if (Etat_module[i] == 3) {
      tension[i] = TensionRefS[i];
    }

  //----------------------------------------------------------------------RELEASE--
    else if (Etat_module[i] == 4) {
      tempsreel[i] = (micros() - TimerStartTop[i]);
      tension[i] = TensionStartTop[i] - (k_timer[i] * tempsreel[i] * float(4095.00 / TimeRefR[i] * 2)); //  LIN
      if      (Choix_forme[i] == 1) {                                     //  LOG
        float k = log(TensionStartTop[i]) * (1 - (k_timer[i] * tempsreel[i] / TimeRefR[i]));
        if (k > 16) {
          tension[i] = 4095;
        } else {
          tension[i] = exp(k);
          tension[i] -= 50;
        }
      }
      else if (Choix_forme[i] == 2) {                                     //  EXP

        tension[i] = (log10((TensionStartTop[i] - (TensionStartTop[i] * (k_timer[i] * tempsreel[i] / TimeRefR[i] * 2))) / (klog * 50))) * ((TensionStartTop[i]) / log10(TensionStartTop[i] / (klog * 50)));
      }
      tension[i] = constrain(tension[i], 0, TensionRefS[i]);
    }
}

//----------------------------------------------
// ENCHAINEMENT DES PHASES ENTRE ELLES
//
//----------------------------------------------

void AUTOMATE() {
  for (int i = 0; i < VCF_VCA ; i++) {
    byte  mem1 = Etat_module[i];
    float mem2 = k_timer[i];
    byte  k = 1; // V3.01
    if (Choix_forme[i] == 1) {
      k = 2; // V3.01
    }

    //------------------------------------------------------------------------------

    if (Etat_module[i] == 0 && ((Etat_gate_in == 1) + (Choix_gate_in[i] == 1) > 0) ) {
      Etat_module[i] = 1;
    }
    else if (Etat_module[i] == 1 && Etat_gate_in == 0 && Choix_gate_in[i] == 0 ) {
      if (tension[i] > TensionRefS[i]) {
        Etat_module[i] = 2;
      } else {
        Etat_module[i] = 4;
      }
    }
    else if (Etat_module[i] == 1 && tension[i] >= 4095) {
      Etat_module[i] = 2;
    }
    else if (Etat_module[i] == 2 && (tension[i] <= TensionRefS[i] || k_timer[i]*tempsreel[i] > TimeRefD[i] / k )) {
      if (Etat_gate_in == 1) {
        Etat_module[i] = 3;
      } else {
        Etat_module[i] = 4;
      }
    }
    else if (Etat_module[i] == 2 && StartTop == 1) {
      Etat_module[i] = 1;
      StartTop = 0;
    }
    else if (Etat_module[i] == 3 && Etat_gate_in == 0) {
      Etat_module[i] = 4;
    }
    else if (Etat_module[i] == 4 && Etat_gate_in == 1 ) {
      Etat_module[i] = 1;
    }
    StartTop = 0;
    if (Etat_module[i] != mem1 || k_timer[i] != mem2) { // INITIALISATION DE L'INSTANT DE BASCULE ENTRE PHASE
      TimerStartTop[i] = micros();
      TensionStartTop[i] = constrain(tension[i], 0, 4095);
    }
  }
}

//----------------------------------------------
// GESTION DES CONTROLES MIDI
//----------------------------------------------

void OnControlChange (byte channel, byte control, byte value) {
  for (int i = 0; i < VCF_VCA ; i++) {
    if (control == MIDI_L[i])
    { LEVEL[i] =  map(value, 0, 127, 0, 4092) ;
    }
    for ( int j = 0 ; j < SW_GTF ; j++ ) {
      if (control == MIDI_GTF[i][j])
      { Etat_SW[i][j] =  map(value, 0, 127, 0, 2) ;
        Choix_gate_in[i] = Etat_SW[0][i];
        Choix_timer[i] = Etat_SW[1][i];
        Choix_forme[i] = Etat_SW[2][i];
      }
    }
    for ( int j = 0 ; j < ADSR ; j++ ) {
      if (control >= 0 && control <= 31) {
        high_byte[channel][control] = value;
      }
      if (control >= 32 && control <= 63) {
        out_value = (high_byte[channel][control - 32] << 7) + value;
        out_value = map(out_value, 0, 16383, 0, 4095);
        if (control == HR_MIDI[i][j]) {
          POT[i][j] =  map(out_value, 0, 4095, 0, 1023) ;
        }
      }
    }
  }
}

//----------------------------------------------
// CALCUL DE LA DUREE DE LA PHASE (ADR) ASSOCIEE AU POTENTIOMETRE
//----------------------------------------------
unsigned long RefTime(unsigned int potar) {
  for (int i = 0; i < VCF_VCA ; i++) {
    unsigned long k = int(potar * (DureeRef[Choix_timer[i]]));;
    k *= float(100000.00 / 1023.00);
    k += Offset[i] + 100;
    return k;
  }
}
// LECTURE PERIODIQUE DES INVERSEURS ET POTENTIOMETRES
//----------------------------------------------
void IHM()
{ for (int i = 0; i < VCF_VCA ; i++) {
    if (Choix_timer[i] == 1) {
      k_timer[i] = 1.00;
    }
    else {
      k_timer[i] = 0.05;
    }
    Offset[i] = int(DureeRef[Choix_timer[i]] * 500);

    TimeRefA[i]     = RefTime(POT[i][0]); TimeRefA[i] /= 2;
    TimeRefD[i]     = RefTime(POT[i][1]);
    TensionRefS[i]  = POT[i][2] * 4 ;
    TimeRefR[i]     = RefTime(POT[i][3]);
  }
}
