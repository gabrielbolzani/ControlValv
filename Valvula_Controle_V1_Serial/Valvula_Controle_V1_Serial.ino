#define buzzer 4
#define m_step 25
#define m_dir 26
#define m_enable 33
#define flow_interrupt 35
#define manual_close 15
#define manual_open 2

int curso = 1133;
int valv_position = 0;
byte m_velocity = 2;
bool trava_motor = false;
byte abs_pos=0;
int Set_point=0;

float n_cicles = 0;
float frequency = 0;
float flow = 0;
double tempo = 0;

void setup () {
  Serial.begin(115200);
  pinMode(buzzer, OUTPUT);
  pinMode(m_step, OUTPUT);
  pinMode(m_dir, OUTPUT);
  pinMode(m_enable, OUTPUT);
  pinMode(flow_interrupt, INPUT);
  pinMode(manual_close, INPUT_PULLDOWN);
  pinMode(manual_open, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(flow_interrupt), treatPulse, FALLING);/*porta 3*/

  digitalWrite(m_step, LOW);
  digitalWrite(m_dir, LOW);
  if (trava_motor) {
    digitalWrite(m_enable, LOW);
  } else {
    digitalWrite(m_enable, HIGH);
  }
  init_valv();
  //tone(buzzer, 800, 400);

}

void calc_flow() {
  detachInterrupt(digitalPinToInterrupt(3));
  if ((millis() - tempo) >= 1000) {
    frequency = 0;
  } else {
    flow = ( (0.000181 * pow(frequency, 3)) - (0.0211 * pow(frequency, 2)) + (7.9 * frequency) - (0));
  }
  attachInterrupt(digitalPinToInterrupt(3), treatPulse, RISING);
}

void treatPulse() {
  frequency = (1.0 / ((millis() - tempo) / 1000.0));
  tempo = millis();
}

void init_valv() {
  int i = 0;
  for (int i = 0; i < 1150; i++) {
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
  Serial.print("Position: "); Serial.print(map(valv_position, 0, 1133, 0, 100)); Serial.println(" %");
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
  //Serial.print("Position: "); Serial.print(map(valv_position, 0, 1133, 0, 100)); Serial.println(" %");
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
int go_to_absolute_position (byte future_pos_porcent) {
  if ((future_pos_porcent >= 0) && (future_pos_porcent <= 100)) {
    int future_pos = map(future_pos_porcent, 0, 100, 0, 1133);
    abs_pos = future_pos_porcent;
    if (valv_position > future_pos) {
      Serial.print("Position: "); Serial.print(map(valv_position, 0, 1133, 0, 100)); Serial.println(" %");
      while (valv_position >= future_pos) {
        close_fn();
        measurements();
      }
      Serial.print("Position: "); Serial.print(map(valv_position, 0, 1133, 0, 100)); Serial.println(" %");
      return 1;//OK
    } else if (valv_position < future_pos) {
      Serial.print("Position: "); Serial.print(map(valv_position, 0, 1133, 0, 100)); Serial.println(" %");
      while (valv_position <= future_pos) {
        open_fn();
        measurements();
      }
      Serial.print("Position: "); Serial.print(map(valv_position, 0, 1133, 0, 100)); Serial.println(" %");
      return 1;//OK
    } else {
      Serial.println("A válvula já se encontra nessa posição!");
      return 2;//Já está nessa posição
    }
  } else {
    Serial.println("A posição digitada é inválida escolha um intervalo entre 0 e 100%");
    return 0;//Não é uma posição válida
  }

}

int go_to_relative_position (byte future_pos_porcent) {
  byte actual_pos_porcent = map(valv_position, 0, 1133, 0, 100);
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

void serialEvent() {
  while (Serial.available()) {
    int dataIn = Serial.parseInt();
    go_to_absolute_position(dataIn);
    Serial.flush();
  }
}

void measurements (){
  calc_flow();
  Serial.print("Position: ");Serial.print(map(valv_position, 0, 1133, 0, 100));
  Serial.print(" ; Flow: ");Serial.print(flow);
  Serial.print(" ; Controller: ");Serial.println(abs_pos);
  //Serial.print(" ; SetPoint: ");Serial.println(Set_point);
}

void loop () {
  //calc_flow();
  measurements();
  if (digitalRead(manual_open)) {
    open_fn();
  }
  if (digitalRead(manual_close)) {
    close_fn();
  }
  if (valv_position > curso) {
    valv_position = curso;
  } else if (valv_position < 0) {
    valv_position = 0;
  }
}
