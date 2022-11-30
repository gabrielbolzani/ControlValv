#define buzzer 4
#define m_step 25
#define m_dir 26
#define m_enable 33
#define flow_interrupt 35
#define manual_close 15
#define manual_open 2


bool controle = false; //0=Manual 1=Automatico
float valv_position = 0;
static float curso = 590;
static float Tolerancia_erro=0.4;//Tolerancia para erro de posição em porcentagem

float tempo_de_amostragem = 500;
byte m_velocity = 3;
bool trava_motor = false;
float abs_pos = 0;
int setPoint = 0;
int Steap = 0;
int n_cicles = 0;
float frequency = 0;
float flow = 0;
double tempo_ultimo_pulso = 0;
double tempo_ultimo_calc = 0;


class PID {
  public:
    double error;
    double sample;
    double lastSample;
    double kP, kI, kD;
    double P, I, D;
    double pid;
    double setPoint;
    long lastProcess;

    PID(double _kP, double _kI, double _kD) {
      kP = _kP;
      kI = _kI;
      kD = _kD;
    }

    void addNewSample(double _sample) {
      sample = _sample;
    }

    void setSetPoint(double _setPoint) {
      setPoint = _setPoint;
    }

    double process() {
      error = setPoint - sample;
      float deltaTime = (millis() - lastProcess) / 1000.0;
      lastProcess = millis();
      P = error * kP;
      I = I + (error * kI) * deltaTime;
      D = (lastSample - sample) * kD / deltaTime;
      lastSample = sample;
      if (I>5){
        I=5;
      }if (I<0){
        I=0;
      }
      pid = P + I + D;
      if (pid>100){
        pid=100;
      }if (pid<0){
        pid=0;
      }
      return pid;
    }
};

PID meuPid(0.125, 0.0006, 0);


void setup() {
  Serial.begin(115200);
  pinMode(buzzer, OUTPUT);
  pinMode(m_step, OUTPUT);
  pinMode(m_dir, OUTPUT);
  pinMode(m_enable, OUTPUT);
  pinMode(flow_interrupt, INPUT);
  pinMode(manual_close, INPUT_PULLDOWN);
  pinMode(manual_open, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(flow_interrupt), treatPulse, FALLING);
  digitalWrite(m_step, LOW);
  digitalWrite(m_dir, LOW);
  if (trava_motor) {
    digitalWrite(m_enable, LOW);
  } else {
    digitalWrite(m_enable, HIGH);
  }
  //init_valv();
  //tone(buzzer, 800, 400);
  //Serial.println("INIT");
}

float Re_map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void close_fn() {
  digitalWrite(m_enable, LOW);
  valv_position--;
  digitalWrite(m_dir, LOW);
  digitalWrite(m_step, HIGH);
  delay(m_velocity);
  digitalWrite(m_step, LOW);
  delay(m_velocity);
  digitalWrite(m_dir, LOW);
  if (trava_motor == false) {
    digitalWrite(m_enable, HIGH);
  }
  //Serial.print("Position: "); Serial.print(map(valv_position, 0, curso, 0, 100)); Serial.println(" %");
}

void open_fn () {
  digitalWrite(m_enable, LOW);
  valv_position++;
  digitalWrite(m_dir, HIGH);
  digitalWrite(m_step, HIGH);
  delay(m_velocity);
  digitalWrite(m_step, LOW);
  delay(m_velocity);
  digitalWrite(m_dir, LOW);
  if (trava_motor == false) {
    digitalWrite(m_enable, HIGH);
  }
  //Serial.print("Position: "); Serial.print(map(valv_position, 0, 1133, 0, 100)); Serial.println(" %");
}

int go_to_absolute_position (float future_pos_porcent) {
  if ((future_pos_porcent >= 0) && (future_pos_porcent <= 100)) {
    float future_pos = Re_map(future_pos_porcent, 0, 100.0, 0.0, curso);
    abs_pos = future_pos_porcent;
    if ((abs(valv_position - future_pos))<Tolerancia_erro){
      return 2;
    }
    if (valv_position > future_pos) {
      measurements();
      while (valv_position >= future_pos) {
        close_fn();
        measurements();
      }
      measurements();
      return 1;//OK
    } else if (valv_position < future_pos) {
      measurements();
      while (valv_position <= future_pos) {
        open_fn();
        measurements();
      }
      measurements();
      return 1;//OK
    } else {
      //Serial.println("A válvula já se encontra nessa posição!");
      return 2;//Já está nessa posição
    }
  } else {
    //Serial.println("A posição digitada é inválida escolha um intervalo entre 0 e 100%");
    return 0;//Não é uma posição válida
  }

}

int go_to_relative_position (float future_pos_porcent) {
  float actual_pos_porcent = Re_map(valv_position, 0.0, curso, 0.0, 100.0);
  if (future_pos_porcent + actual_pos_porcent > 100) {
    go_to_absolute_position(100);//Saturada em 100%
    return 3;
  } else if (future_pos_porcent + actual_pos_porcent < 0) {
    go_to_absolute_position(0);
    return 2;//Saturada em 0%
  } else {
    go_to_absolute_position(actual_pos_porcent + future_pos_porcent);
    return 1;//OK
  }
}

void calc_flow() {
  if ((millis() - tempo_ultimo_calc) >= tempo_de_amostragem) {
    //Serial.println("CALC");
    frequency = (n_cicles / (tempo_de_amostragem / 1000));
    //flow = ( (0.000181 * pow(frequency, 3)) - (0.0211 * pow(frequency, 2)) + (7.9 * frequency) - (0));
    flow = ((7.6724 * frequency) - 7.727);
    //Serial.print("FLOW: "); Serial.println(flow);
    if (flow < 0) {
      flow = 0;
    }
    n_cicles = 0;
    tempo_ultimo_calc = millis();
    if (controle) {
      meuPid.addNewSample(flow);
      //Adicionando uma nova leitura
    }
  }
}

void treatPulse() {
  n_cicles++;
  //tempo_ultimo_pulso = millis();
}

void serialEvent() {
  String inputString = "";
  bool stringComplete = false;
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
  if (stringComplete) {
    String value;
    int i = 1;
    while (inputString[i] != '\n') {
      value += inputString[i];
      i++;
    }
    //Serial.println(inputString);
    switch (inputString[0]) {
      case 'S':
      case 's':
        //Controle automatico  - Setpoint
        controle = true;
        //Serial.print("SetPint "); Serial.println(value.toFloat());
        setPoint = value.toFloat();
        meuPid.setSetPoint(setPoint);
        break;
      case 'R':
      case 'r':
        //Controle Manual  - posição relativa %
        controle = false;
        //Serial.print("Posição relativa: "); Serial.println(value.toFloat());
        go_to_relative_position(value.toFloat());
        break;
      case 'A':
      case 'a':
        //Controle Manual  - posição relativa %
        controle = false;
        //Serial.print("Posição Absoluta: "); Serial.println(value.toFloat());
        go_to_absolute_position(value.toFloat());
        break;
      case 'I':
      case 'i':
        controle = false;
        init_valv();
        go_to_absolute_position(value.toFloat());
        break;
      case 'L':
      case 'l':
        controle = false;
        measurements();
        break;
      default:
        Serial.println("Comando não reconhecido");
        break;
    }
    inputString = "";
  }
  stringComplete = false;
}

void init_valv() {
  int i = 0;
  for (int i = 0; i < 600; i++) {
    digitalWrite(m_enable, LOW);
    digitalWrite(m_dir, LOW);
    digitalWrite(m_step, HIGH);
    delay(m_velocity);
    digitalWrite(m_step, LOW);
    delay(m_velocity);
    digitalWrite(m_dir, LOW);
    if (trava_motor == false) {
      digitalWrite(m_enable, HIGH);
    }
  }
  valv_position = 0;
  //Serial.print("Position: "); Serial.print(map(valv_position, 0, curso, 0, 100)); Serial.println(" %");
}


void measurements() {
  calc_flow();
  Serial.print("Controller: "); Serial.print(abs_pos);
  Serial.print("; Position: "); Serial.print((float)Re_map(valv_position, 0.0, curso, 0.0, 100.0));
  Serial.print(" ; Flow: "); Serial.print(flow);
  if (controle){
  Serial.print(" ; SetPoint: "); Serial.print(meuPid.setPoint);
  Serial.print(" ; P: "); Serial.print(meuPid.P);
  Serial.print(" ; I: "); Serial.print(meuPid.I);
  Serial.print(" ; D: ");Serial.print(meuPid.D);
  Serial.print(" ; PID: ");Serial.print(meuPid.pid);
  }
  Serial.print("\n");
}

void loop () {
  //calc_flow();
  if (controle) {
    measurements();
    go_to_absolute_position(meuPid.process());
  }
  if (digitalRead(manual_open)) {
    open_fn();
    measurements();
  }
  if (digitalRead(manual_close)) {
    close_fn();
    measurements();
  }
  if (valv_position > curso) {
    valv_position = curso;
  } else if (valv_position < 0) {
    valv_position = 0;
  }
}
