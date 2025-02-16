#include <Servo.h>
int c = 0 ;
// Сервы
#define SERVO_UP_PIN 11
#define SERVO_HVATALKA_PIN 10

// Расстояние
#define DISTANCE_PIN A0

// Линия
#define LINE_1_PIN A1
#define LINE_2_PIN A2
#define LINE_3_PIN A3
#define LINE_4_PIN A4
#define LINE_5_PIN A5

//#define LIMIT_LINE 422

//=============================================
///   Для колесных моторов настройки
//Ширина импульсов для для управления
#define LEFT_MOTOR_DIR_PIN      7
#define LEFT_MOTOR_SPEED_PIN    6
#define RIGHT_MOTOR_DIR_PIN     4
#define RIGHT_MOTOR_SPEED_PIN   5

#define DIR_FORWARD LOW
#define DIR_REVERSE HIGH


/*
#define SPEED_MIN_PWM_WIDTH_L     255
#define SPEED_MAX_PWM_WIDTH_L     255
#define SPEED_MIN_PWM_WIDTH_R     255
#define SPEED_MAX_PWM_WIDTH_R     255
//
#define STARTUP_TIME            55
#define DEGREE_TO_TIME_K        14.1
#define SPEED_TO_TIME_K         0.01
#define MM_TO_TIME_K            5.9
*/
#define SPEED_MIN_PWM_WIDTH_L     210
#define SPEED_MAX_PWM_WIDTH_L     255
#define SPEED_MIN_PWM_WIDTH_R     210
#define SPEED_MAX_PWM_WIDTH_R     255
//
#define STARTUP_TIME            55
#define DEGREE_TO_TIME_K        13
#define SPEED_TO_TIME_K         0.01
#define MM_TO_TIME_K            5.9




#define SPEED                   1

// Цвет
#define COLOR_S0 2
#define COLOR_S1 8
#define COLOR_S2 13
#define COLOR_S3 12
#define COLOR_OUT 3

#define HVATALKA_ANGLE 55
#define UPPER_ANGLE 65 // переменная для угла поворота микро серво

#define COLOR_GET_REPEATS 3

Servo upper{};
Servo hvatalka{};

constexpr const float redFrequency = 0.0f;
constexpr const float greenFrequency = 0.0f;
constexpr const float blueFrequency = 0.0f;

uint32_t periods[COLOR_GET_REPEATS] = {};
uint32_t timer = 0;
size_t period_pos = 0;

void setup() {
  // Motors
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_SPEED_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_SPEED_PIN, OUTPUT);
  analogWrite(LEFT_MOTOR_SPEED_PIN, 0);
  analogWrite(RIGHT_MOTOR_SPEED_PIN, 0);
  pinMode(13, OUTPUT);
  // Servos
  upper.attach(SERVO_UP_PIN);
  hvatalka.attach(SERVO_HVATALKA_PIN);

  upper.write(0);
  hvatalka.write(55);

  // Distance
  pinMode(DISTANCE_PIN, INPUT);

  // lines
  pinMode(LINE_1_PIN, INPUT);
  pinMode(LINE_2_PIN, INPUT);
  pinMode(LINE_3_PIN, INPUT);
  pinMode(LINE_4_PIN, INPUT);
  pinMode(LINE_5_PIN, INPUT);

  // Setting the outputs
  pinMode(COLOR_S0, OUTPUT);
  pinMode(COLOR_S1, OUTPUT);
  pinMode(COLOR_S2, OUTPUT);
  pinMode(COLOR_S3, OUTPUT);

  // Setting the sensorOut as an input
  pinMode(COLOR_OUT, INPUT);

  // Setting frequency scaling to off
  digitalWrite(COLOR_S0, HIGH);
  digitalWrite(COLOR_S1, LOW);

  // Debug
  Serial.begin(115200);
  while (!Serial);
}

void signal_irq_handler() {
  if (digitalRead(COLOR_OUT) == HIGH) {
    timer = micros();
    return;
  }

  periods[period_pos++] = micros() - timer;
  if (period_pos == COLOR_GET_REPEATS) {
    detachInterrupt(digitalPinToInterrupt(COLOR_OUT));
    digitalWrite(COLOR_S0, LOW);
    digitalWrite(COLOR_S1, LOW);
  }
}

float get_distance() {
  float voltage = analogRead(DISTANCE_PIN) * 0.0048828125;
  return 13.0f * pow(voltage, -1.0);
}


float get_color_frequence() {
  period_pos = 0;
  attachInterrupt(digitalPinToInterrupt(COLOR_OUT), signal_irq_handler, CHANGE);
  digitalWrite(COLOR_S0, HIGH);
  digitalWrite(COLOR_S1, HIGH);

  while ((digitalRead(COLOR_S0) == HIGH) || (digitalRead(COLOR_S1) == HIGH)) {}
    
  float freq = 0.0f;

  for (size_t i = 0; i < COLOR_GET_REPEATS; i++) {
    freq += (1.0f / (periods[i] / 500000.0f));
  }

  return freq / (float)COLOR_GET_REPEATS;
}

//======================================================================================
//======================================================================================
//======================================================================================
//======================================================================================

//НЕ ЗАРАБОТАЛО!!!
int detectColor() {
  digitalWrite(COLOR_S0, LOW);
  digitalWrite(COLOR_S1, HIGH);

  // activating red photodiodes to read
  digitalWrite(COLOR_S2, LOW);
  digitalWrite(COLOR_S3, LOW);
  int frequency = pulseIn(COLOR_OUT, LOW);
  int R = frequency;
  Serial.print("Red = ");
  Serial.print(frequency);//printing RED color frequency
  Serial.print("   ");
  delay(50);

  // activating blue photodiodes to read
  digitalWrite(COLOR_S2, LOW);
  digitalWrite(COLOR_S3, HIGH);
  frequency = pulseIn(COLOR_OUT, LOW);
  int B = frequency;
  Serial.print("Blue = ");
  Serial.print(frequency);
  Serial.println("   ");

  // activating green photodiodes to read
  digitalWrite(COLOR_S2, HIGH);
  digitalWrite(COLOR_S3, HIGH);
  // Reading the output frequency
  frequency = pulseIn(COLOR_OUT, LOW);
  int G = frequency;
  Serial.print("Green = ");
  Serial.print(frequency);
  Serial.print("   ");
  delay(50);

  delay(50);
  digitalWrite(COLOR_S0, LOW);
  digitalWrite(COLOR_S1, LOW);


  return 1;
}



short GetLineState()
{
  short state1 = 0, state2 = 1;
  
  while (state1 != state2)
  {
    //Читаем 2 раза все датчики и проверяем, что оба раза прочитано одно и тоже состояние, возможно позже добавим задержку между замерами
    state1 = (digitalRead(LINE_1_PIN) | (digitalRead(LINE_2_PIN) << 1) | (digitalRead(LINE_3_PIN) << 2) | (digitalRead(LINE_4_PIN) << 3) | (digitalRead(LINE_5_PIN) << 4));
    delay(2);
    state2 = (digitalRead(LINE_1_PIN) | (digitalRead(LINE_2_PIN) << 1) | (digitalRead(LINE_3_PIN) << 2) | (digitalRead(LINE_4_PIN) << 3) | (digitalRead(LINE_5_PIN) << 4));
  }
  

  return (state1);
}

//Управление моторами 0 - стоять, 1..100 - скорость, знак направление
void MotorControlLR(short speedLeft, short speedRight)
{
  short lDir, rDir;
  short lPwm, rPwm;
  
  //разберемся с направлением ЛЕВОГО
  if (speedLeft < 0)
  {
    lDir = DIR_REVERSE;
  }
  else
  {
    lDir = DIR_FORWARD;
  }

  //теперь расчитаем ширину импульсов для ЛЕВОГО
  if (speedLeft == 0)
  {
    lPwm = 0;
  }
  else
  {
    lPwm = SPEED_MIN_PWM_WIDTH_L + ((abs(speedLeft) * (SPEED_MAX_PWM_WIDTH_L - SPEED_MIN_PWM_WIDTH_L)) / 100);
  }

  //разберемся с направлением ПРАВОГО
  if (speedRight < 0)
  {
    rDir = DIR_REVERSE;
  }
  else
  {
    rDir = DIR_FORWARD;
  }

  //теперь расчитаем ширину импульсов для ПРАВОГО
  if (speedRight == 0)
  {
    rPwm = 0;
  }
  else
  {
    rPwm = SPEED_MIN_PWM_WIDTH_R + ((abs(speedRight) * (SPEED_MAX_PWM_WIDTH_R - SPEED_MIN_PWM_WIDTH_R)) / 100);
  }

  //Теперь установим посчитанные значения 
  digitalWrite(LEFT_MOTOR_DIR_PIN, lDir);
  digitalWrite(RIGHT_MOTOR_DIR_PIN, rDir);
  analogWrite(LEFT_MOTOR_SPEED_PIN, lPwm);
  analogWrite(RIGHT_MOTOR_SPEED_PIN, rPwm);
}

int CalcTimeForRotate(int degree, int speed)
{
//ТРЕБУЕТСЯ КАЛИБРОВКА!!!!!!!!!!
  float time = (float)(abs(degree)) * DEGREE_TO_TIME_K;
  //Для большей скорости меньше время нужно
  if (speed != 0)
  {
    //Пока уберу
    //time /= (float)(abs(speed)) * SPEED_TO_TIME_K;
  }
  return time;
}

int CalcTimeForStraight(int length, int speed)
{
//ТРЕБУЕТСЯ КАЛИБРОВКА!!!!!!!!!!
  float time = (float)(abs(length)) * MM_TO_TIME_K;
  //Для большей скорости меньше время нужно
  if (speed != 0)
  {
    //Пока уберу
    //time /= (float)(abs(speed)) * SPEED_TO_TIME_K;
  }
  return time;
}

//Довернуть на угол (через торможение одного)
void Steer(int degree, int speed)
{
  int time = CalcTimeForRotate(degree, speed);
  if (degree < 0)
  {
    MotorControlLR(0, speed);
  }
  else if (degree > 0)
  {
    MotorControlLR(speed, 0);
  }

  delay(time);
  MotorControlLR(speed, speed);
}


//повернуться на угол (через движение в разные стороны)
void Rotate(int degree, int speed)
{
  float time = STARTUP_TIME + CalcTimeForRotate(degree, speed) / 2;
  if (degree < 0)
  {
    MotorControlLR(-speed, speed);
  }
  else if (degree > 0)
  {
    MotorControlLR(speed, -speed);
  }
  
  delay(time);
  MotorControlLR(0, 0);
}

unsigned long prevTime = 0;
unsigned long leftCornerTime = 0, rightCornerTime = 0;    //Время когда был последний поворот
unsigned int leftCornerCounter = 0, rightCornerCounter = 0;   //Сколько раз был отмечен поворот
int searchCounter = 0, returnCounter = 0;

unsigned int noLineCounter = 0;

#define STATE_START   0   //Начальная фаза, чтобы не ехал пока не поставили на трассу
#define STATE_SEARCH  1   //Фаза поиска финиша
#define STATE_RETURN  2   //Фаза возврата зеленого на старт
#define STATE_FINISH  3   //Фаза езды до финиша после зеленого
#define STATE_STOP    4   //Фаза окончания прохождения


#define CORNER_EXP_TIME   (20 * MM_TO_TIME_K)     //20мм дальше надо возвращаться
#define FILTER_NUM    2   //Минимальное количество получения информации от датчиков линии о повороте

int phase = STATE_START;   //Тут текущую фазу храним

void loop() 
{
  short state = GetLineState();       //Получаем состояние датчиков линии
  unsigned int curTime = millis();    //Запоминаем текущее время в миллисекундах

  //Линия посередине - едем прямо
  if (state == B11011)
  {
    if (phase == STATE_START)       //Переходим в состояние поиска финиша по лабиринту
    {  
      phase = STATE_SEARCH;
      delay(1000);                  //Отложенный старт - процедура показа, что мы готовы - СДЕЛАТЬ
      curTime = millis();
      Serial.print("START!!!");
    }
    Steer(0, SPEED);                //Едем прямо
    //delay(MM_TO_TIME_K * 3);
  }

  //Можно направо - запомним
  if (state == B11111)           //Тупик?
  {
    Serial.println(state, BIN);
    noLineCounter++;
    if (noLineCounter > 10)
    {
      Serial.println("T");
    }
  }
  else
  {
    noLineCounter = 0;
  }



  //Можно направо - запомним
  if ((state & B11) == 0)           //Запомним время, когда обнаружен правый поворот
  {
    Serial.println(state, BIN);
    rightCornerCounter++;
    if (rightCornerCounter > FILTER_NUM)
    {
      Serial.println("R");
      rightCornerTime = curTime;      //Запоминаем время
    }
    state |= B11;                   //Сбросим признак для правых датчиков, чтобы не корректировать
    state &= B11011;                //КОСТЫЛЬ чтобы не корректировали курс, по пересечению
  }
  else
  {
    rightCornerCounter = 0;
  }

  //Можно налево - запомним
  if ((state & B11000) == 0)        //Запомним время, когда обнаружен левый поворот
  {
    Serial.println(state, BIN);
    leftCornerCounter++;
    if (leftCornerCounter > FILTER_NUM)
    {
      leftCornerTime = curTime;      //Запоминаем время
      Serial.println("L");
    }
    state |= B11000;                //Сбросим признак для левых датчиков, чтобы не корректировать 
    state &= B11011;                //КОСТЫЛЬ чтобы не корректировали курс, по пересечению
  }
  else
  {
    leftCornerCounter = 0;
  }


  //Если мы в фазе езды
  if ((phase == STATE_SEARCH) || (phase == STATE_FINISH) || (phase == STATE_RETURN))
  {
    //Кончилась трасса(был поворот не в приоритетную сторону или тупик)?
    if ((state == B11111) && (noLineCounter > 10))
    {
      Serial.println(state, BIN);
      //Если мы ищим финиш или едем на него и только что проехали левый поворот
      if (((phase == STATE_SEARCH) || (phase == STATE_FINISH)) && (curTime - leftCornerTime < CORNER_EXP_TIME))
      {
        
        Serial.println("FL");
        
        Steer(0,0);       //Стоп
        delay(100);        //Ждем остановки
        Steer(-30, SPEED);
        Rotate(-45, SPEED); //Поворачиваем
        delay(20);        //Ждем остановки
        while ((GetLineState() & B00100) != 0)    //цикл докручивания до линии посередине
        {
          Rotate(-3, SPEED);
          delay(30);        //Ждем остановки
        }
        Steer(0, SPEED);    //Погнали прямо

        leftCornerTime = 0;       //Отработали - сбросим
        leftCornerCounter = 0;
      }
      //Если мы ищим возвращаемся с грузом и только что проехали правый поворот
      else if ((phase == STATE_RETURN) && (curTime - rightCornerTime < CORNER_EXP_TIME))
      {
        Serial.println("FR");
        
        Steer(0,0);       //Стоп
        delay(100);        //Ждем остановки
        Steer(30, SPEED);
        Rotate(45, SPEED); //Поворачиваем
        delay(20);        //Ждем остановки
        while ((GetLineState() & B00100) != 0)    //цикл докручивания до линии посередине
        {
          Rotate(3, SPEED);
          delay(30);        //Ждем остановки
        }
        Steer(0, SPEED);    //Погнали прямо

        rightCornerTime = 0;       //Отработали - сбросим
        rightCornerCounter = 0;
      }
      //Если повороты были давно, то это тупик, просто разварачиваемся на 180
      else
      {
        Serial.println("ROTATATE");
        Steer(0, 0);
        delay(100);
        Rotate(150, SPEED);
        delay(100);        //Ждем остановки
        while ((GetLineState() & B00100) != 0)        //цикл докручивания до линии посередине
        {
          Rotate(3, SPEED);
          delay(30);        //Ждем остановки
         }
        Steer(0, SPEED);
      }
    }
    //Если мы ищим финиш или едем на него и обнаружили правый поворот
    else if (((phase == STATE_SEARCH) || (phase == STATE_FINISH)) && (curTime - rightCornerTime < CORNER_EXP_TIME))
    {
      Serial.println("FR");
      delay(100);       //Проедем еще немного вперед ~поллинии
      Steer(0,0);       //Стоп
      delay(100);        //Ждем остановки
      Steer(70, SPEED); //Поворачиваем, но не до конца
      Steer(0,0);       //Стоп
      delay(100);        //Ждем остановки 
      while ((GetLineState() & B00100) != 0)      //цикл докручивания до линии посередине
      {
        Rotate(3, SPEED);
        delay(30);        //Ждем остановки
       }
       Steer(0, SPEED);    //Погнали

      rightCornerTime = 0;       //Отработали - сбросим
      rightCornerCounter = 0;
   }
    //Если мы ищим возвращаемся с грузом и обнаружили левый поворот
    else if ((phase == STATE_RETURN) && (curTime - leftCornerTime < CORNER_EXP_TIME))
    {
      Serial.println("FL");
      delay(100);       //Проедем еще немного вперед ~поллинии
      Steer(0,0);       //Стоп
      delay(100);        //Ждем остановки
      Steer(-70, SPEED); //Поворачиваем, но не до конца
      Steer(0,0);       //Стоп
      delay(100);        //Ждем остановки 
      while ((GetLineState() & B00100) != 0)      //цикл докручивания до линии посередине
      {
        Rotate(-3, SPEED);
        delay(30);        //Ждем остановки
      }
      Steer(0, SPEED);    //Погнали

      leftCornerTime = 0;       //Отработали - сбросим
      leftCornerCounter = 0;
    }
    


    //Съезжаем влево?
    if ((state & B10) == 0)
    {
      Serial.print("CR ");
      Serial.println(state, BIN);
      Steer(3, SPEED);       //Подрулим вправо
    }
    //Съезжаем вправо?
    if ((state & B1000) == 0)
    {
      Serial.print("CL ");
      Serial.println(state, BIN);
      Steer(-3, SPEED);       //Подрулим влево
    }
  }

//ДОЕХАЛИ ДО ЦИЛИНДРА
 if (get_distance() < 7.5)
 {
  Steer(0, 0); 
  delay(500);
  c-=-1;
  if (c==1){
    Steer(50, SPEED);   
    delay(1200);
    Steer(-90, SPEED); 
    delay(1000);
    Steer(40, SPEED);
    delay(500);
  }
  if (c==2){
    // hvatalka.write(HVATALKA_ANGLE);
    // delay(1000);
     upper.write(65);
     delay(1000);
    hvatalka.write(0);
    delay(500);
    upper.write(0);
    delay(500);

    Rotate(90, SPEED);
    Steer(0, SPEED);
    delay(100);
    Steer(0, 0);
    upper.write(UPPER_ANGLE);
    delay(500);
    hvatalka.write(HVATALKA_ANGLE);
    delay(500);
    upper.write(0);
    delay(500);

    Steer(0, -SPEED);
    delay(100);
    Rotate(-90, SPEED);
    delay(100);
    Steer(0, SPEED);   
  }
  if (c==3){
     upper.write(65);
     delay(1000);
    hvatalka.write(0);
    delay(500);
    upper.write(0);
    delay(500);

    Rotate(90, SPEED);
    Steer(0, SPEED);
    delay(100);
    Steer(0, 0);
    upper.write(UPPER_ANGLE);
    delay(500);
    hvatalka.write(HVATALKA_ANGLE);
    delay(500);
    upper.write(0);
    delay(500);

    Steer(0, -SPEED);
    delay(100);
    Rotate(-90, SPEED);
    delay(100);
    Steer(0, SPEED);    
  }
  if (c==4){
    Steer(-45, SPEED);   
    delay(1200);
    Steer(80, SPEED); 
    delay(1200);
    Steer(-40, SPEED);
    delay(500);   
  }

  if (c==5){
     upper.write(65);
     delay(2000);
    hvatalka.write(0);
    delay(1000);
    upper.write(0);
    delay(1000);  
   Steer(0, -SPEED);
    delay(100);
    Rotate(-180, SPEED);
    delay(100);
    Steer(0, SPEED);  
  }
  if (c==6){
    Steer(45, SPEED);   
    delay(1200);
    Steer(-80, SPEED); 
    delay(1200);
    Steer(40, SPEED);
    delay(500);   
  }
  if (c==7){
    Steer(-50, SPEED);   
    delay(1200);
    Steer(90, SPEED); 
    delay(1000);
    Steer(-40, SPEED);
    delay(500);
  }
 // g+=1;
  //if (g==5){
    //Supper.write(angle); 
    //delay(1000); 
    //hvatalka.write(pan); 
    //delay(1000); 
    //upper.write(40); 
    //delay(2000); 
    //hvatalka.write(0); 
    //delay(1000); 
    //upper.write(0); 
    //delay(1000); 

  //}
  //else{

//  }

//    Serial.print("Color freq: ");
//    Serial.println((uint32_t)get_color_frequence());
/*
    upper.write(UPPER_ANGLE);
    delay(1000);
    // hvatalka.write(HVATALKA_ANGLE);
    // delay(1000);
    // upper.write(40);
    // delay(2000);
    hvatalka.write(0);
    delay(1000);
    upper.write(0);
    delay(1000);

    Rotate(90, SPEED);
    Steer(0, SPEED);
    delay(100);
    Steer(0, 0);
    upper.write(UPPER_ANGLE);
    delay(1000);
    hvatalka.write(HVATALKA_ANGLE);
    delay(1000);
    upper.write(0);
    delay(1000);

    Steer(0, -SPEED);
    delay(100);
    Rotate(-90, SPEED);
    delay(100);
    Steer(0, SPEED);
*/

  }

/*

//==========Это для отладки=============
  int ch = Serial.read(); 

  if (ch == 'l')
  {
    for(int i = 0; i < 6; i++)
    {
      Rotate(-15, SPEED);
      delay(500);
    }
  }
  if (ch == 'L')
  {
    Rotate(-90, SPEED);
    delay(500);
  }

  if (ch == 'r')
  {
    for(int i = 0; i < 6; i++)
    {
      Rotate(15, SPEED);
      delay(500);
    }
  }
  if (ch == 'R')
  {
    Rotate(90, SPEED);
    delay(500);
  }

  if (ch == 'D')
  {
    Steer(0, SPEED);
    delay(500);
    Steer(90,SPEED);
    delay(500);
    Steer(0, 0);
  }

  if (ch == 'A')
  {
    Steer(0, SPEED);
    delay(500);
    Steer(-90,SPEED);
    delay(500);
    Steer(0, 0);
  }

  if (ch == 'W')
  {
    Steer(0, SPEED);
    delay(CalcTimeForStraight(100, SPEED));
    Steer(0, 0);
  }

  if (ch == 'S')
  {
    Steer(0, -SPEED);
    delay(CalcTimeForStraight(100, SPEED));    //1000 - 125
    Steer(0, 0);
  }

*/
}