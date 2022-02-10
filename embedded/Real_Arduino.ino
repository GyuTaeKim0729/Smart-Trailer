// Driving Motor
#define Direction_Pin 2 // 구동 방향(전진, 후진) 핀
#define Speed_PWM_Pin 3 // 구동 속도 핀

// Front/Rear Steering Motor
#define Front_Steering_DIR_Pin 8 // 전륜 조향 핀
#define Front_Steering_PWM_Pin 9 // 전륜 속도 핀
#define Rear_Steering_DIR_Pin 11 // 후륜 조향 핀
#define Rear_Steering_PWM_Pin 12 // 후륜 속도 핀
#define Front_Steering_Speed 150 // 조향 모터 속도 (0~255)
#define Rear_Steering_Speed 150  // 조향 모터 속도 (0~255)
#define Angle_delay_front 30     // 1도 전륜 변환 시간
#define Angle_delay_rear 55      // 1도 후륜 변환 시간
#define Angle_delay_term 40      // 각도간의 여유 시간

//50 150
// Encoder
#define Middle_Encoder_Pin A0 // 굴절각 엔코더 핀

// Microsonic Sensors
#define S_Sensor_Num 6 // 초음파 개수
#define TRIG 24        // 트리거
#define ECHO_F 26      // 전방 초음파 센서 핀
#define ECHO_R 27      // 후방 초음파 센서 핀
#define ECHO_A_L 28    // 전좌측방 초음파 센서 핀
#define ECHO_A_R 29    // 전우측방 초음파 센서 핀
#define ECHO_B_L 30    // 후좌측방 초음파 센서 핀
#define ECHO_B_R 31    // 후우측방 초음파 센서 핀

// Serial Communication - 수신 시 필요한 데이터 임시저장 변수
String ch;
char front_angle_str[4] = {0};
char rear_angle_str[4] = {0};
char speed_str[4] = {0};

// Receive Data PC(Python) -> Arduino 데이터
// 90가 전진방향, 90이하는 왼쪽, 90이상은 오른쪽
int Receive_Front_Angle = 90; // 수신한 전륜 조향각
int Receive_Rear_Angle = 90;  // 수신한 후륜 조향각e
int Receive_Speed = 0;        // 수신한 속도
int Receive_Direction = 0;    // 수신한 구동 방향

// Car`s State
int Front_Angle = 90; // 현재 전륜 조향각
int Rear_Angle = 90;  // 현재 후륜 조향각
int Speed = 0;        // 현재 속도
int Direction = 0;    // 현재 구동 방향

int Sonic_Count = 0;                                                         // 초음파 순서 위한 변수
double Distances[S_Sensor_Num] = {200.0, 200.0, 200.0, 200.0, 200.0, 200.0}; // 0: E_F / 1: E_R / 2: A_L / 3: A_R / 4: B_L / 5:B_R
int Distances_Chance[S_Sensor_Num] = {0, 0, 0, 0, 0, 0};                     // 잘 못 수신한 것을 필터하기 위한 배열

int Middle_Angle = 180; // 현재 굴절각
int Middle_Angles[360]; // 굴절각 초기화 당시 저장되는 세팅 배열

volatile boolean camera_on = false; // 카메라 연결 확인

//------------------- Serial Port--------------------

// 'A' 수신시 카메라 연동 성공
// 수신 안될 경우 계속해서 'N' 전송
void Serial_Start()
{
  if (ch[0] == 'A')
  {
    Serial.println('Y');
    camera_on = true;
    ch = "";
  }
  else
    Serial.println('N');
}

// 변경할 자동사 상태 정보 수신
void Serial_Receive()
{
  while (Serial.available())
  {
    char temp = Serial.read();

    ////////////////////////////////////////////////////////////////////////////////

    if (temp == 'a')
    {
      digitalWrite(Front_Steering_DIR_Pin, HIGH);
      analogWrite(Front_Steering_PWM_Pin, Front_Steering_Speed);
      delay(Angle_delay_front);
      analogWrite(Front_Steering_PWM_Pin, 0);
      delay(Angle_delay_term);
    }
    else if (temp == 'd')
    {
      digitalWrite(Front_Steering_DIR_Pin, LOW);
      analogWrite(Front_Steering_PWM_Pin, Front_Steering_Speed);
      delay(Angle_delay_front);
      analogWrite(Front_Steering_PWM_Pin, 0);
      delay(Angle_delay_term);
    }
    else if (temp == 't') // right
    {
      for (int i = 0; i < 30; i++)
      {
        digitalWrite(Front_Steering_DIR_Pin, LOW);
        analogWrite(Front_Steering_PWM_Pin, Front_Steering_Speed);
        delay(Angle_delay_front);
        analogWrite(Front_Steering_PWM_Pin, 0);
        delay(Angle_delay_term);
      }
    }
    else if (temp == 'r') // Left
    {
      for (int i = 0; i < 30; i++)
      {
        digitalWrite(Front_Steering_DIR_Pin, HIGH);
        analogWrite(Front_Steering_PWM_Pin, Front_Steering_Speed);
        delay(Angle_delay_front);
        analogWrite(Front_Steering_PWM_Pin, 0);
        delay(Angle_delay_term);
      }
    }

    ////////////////////////////////////////////////////////////////////////////////

    if (temp == 'A') // 카메라 연동 시작
      ch = "A";
    else if (temp == 'B') // 카메라 연동 종료
    {
      Receive_Speed = 0;
      Car_Control(); // 조향 초기화

      camera_on = false; // 카메라 연동 종료
      ch = "";
    }
    else if (temp == 'C')
      ch = "C"; // 입력 데이터 시작 인지
    else if (ch[0] == 'C')
      ch.concat(temp); // 입력 데이터 합치기
    else if (temp == 'D')
    {
      digitalWrite(Front_Steering_DIR_Pin, HIGH);
      analogWrite(Front_Steering_PWM_Pin, Front_Steering_Speed);
      delay(Angle_delay_front);
      analogWrite(Front_Steering_PWM_Pin, 0);
      delay(Angle_delay_term);
      ch = "";
    }
    else if (temp == 'E')
    {
      digitalWrite(Front_Steering_DIR_Pin, LOW);
      analogWrite(Front_Steering_PWM_Pin, Front_Steering_Speed);
      delay(Angle_delay_front);
      analogWrite(Front_Steering_PWM_Pin, 0);
      delay(Angle_delay_term);
      ch = "";
    }
    else if (temp == 'F')
    {
      digitalWrite(Rear_Steering_DIR_Pin, HIGH);
      analogWrite(Rear_Steering_PWM_Pin, Rear_Steering_Speed);
      delay(Angle_delay_rear);
      analogWrite(Rear_Steering_PWM_Pin, 0);
      delay(Angle_delay_term);
      ch = "";
    }
    else if (temp == 'G')
    {
      digitalWrite(Rear_Steering_DIR_Pin, LOW);
      analogWrite(Rear_Steering_PWM_Pin, Rear_Steering_Speed);
      delay(Angle_delay_rear);
      analogWrite(Rear_Steering_PWM_Pin, 0);
      delay(Angle_delay_term);
      ch = "";
    }
    else if (temp == 'H')
    {
      digitalWrite(Front_Steering_DIR_Pin, HIGH);
      digitalWrite(Rear_Steering_DIR_Pin, LOW);
      analogWrite(Front_Steering_PWM_Pin, Front_Steering_Speed);
      analogWrite(Rear_Steering_PWM_Pin, Rear_Steering_Speed);
      delay(Angle_delay_front);
      analogWrite(Front_Steering_PWM_Pin, 0);
      delay(Angle_delay_rear - Angle_delay_front);
      analogWrite(Rear_Steering_PWM_Pin, 0);
      delay(Angle_delay_term);
      ch = "";
    }
    else if (temp == 'I')
    {
      digitalWrite(Front_Steering_DIR_Pin, LOW);
      digitalWrite(Rear_Steering_DIR_Pin, HIGH);
      analogWrite(Front_Steering_PWM_Pin, Front_Steering_Speed);
      analogWrite(Rear_Steering_PWM_Pin, Rear_Steering_Speed);
      delay(Angle_delay_front);
      analogWrite(Front_Steering_PWM_Pin, 0);
      delay(Angle_delay_rear - Angle_delay_front);
      analogWrite(Rear_Steering_PWM_Pin, 0);
      delay(Angle_delay_term);
      ch = "";
    }
    else if (temp == 'R')
    {
      Middle_Angle_init();    // 굴절각 초기화
      Middle_Angle_Control(); // 굴절각 제어
      ch = "";
    }
    else
      ch = "";
  }
}

// 수신한 데이터 처리
void Serial_Process()
{
  if (ch[0] == 'C' && ch.length() == 11) // C, 090, 090, 000, 0 / C, 전륜 조향각, 후륜 조향각, 속도, 구동 방향
  {
    ch.substring(1, 4).toCharArray(front_angle_str, 4);
    ch.substring(4, 7).toCharArray(rear_angle_str, 4);
    ch.substring(7, 10).toCharArray(speed_str, 4);
    Receive_Front_Angle = atoi(front_angle_str); // 수신 전륜 조향각
    Receive_Rear_Angle = atoi(rear_angle_str);   // 수신 후륜 조향각
    Receive_Speed = atoi(speed_str);             // 수신 속도
    Receive_Direction = ch[10] - '0';            // 수신 구동 방향
    ch = "";
  }
}

// 현재의 자동차 상태 정보 전송
// 090, 090, 180, 000, 0, 000, 000, 000, 000, 000, 000 - 전륜 조향각, 후륜 조향각, 굴절각, 속도, 구동 방향, 초음파 6개
void Serial_Send()
{
  String Data, F_Angle, R_Angle, M_Angle, N_Speed, Dir;

  if (Front_Angle < 100)
    F_Angle = "0" + String(Front_Angle);
  else
    F_Angle = String(Front_Angle);

  if (Rear_Angle < 100)
    R_Angle = "0" + String(Rear_Angle);
  else
    R_Angle = String(Rear_Angle);

  if (Middle_Angle < 10)
    M_Angle = "00" + String(Middle_Angle);
  else if (Middle_Angle < 100)
    M_Angle = "0" + String(Middle_Angle);
  else
    M_Angle = String(Middle_Angle);

  if (Speed < 10)
    N_Speed = "00" + String(Speed);
  else if (Speed < 100)
    N_Speed = "0" + String(Speed);
  else
    N_Speed = String(Speed);

  Dir = String(Direction);

  Data = F_Angle + R_Angle + M_Angle + N_Speed + Dir; // 정리한 데이터 합치기

  for (int i = 0; i < S_Sensor_Num; i++) // 초음파 데이터 합치기
  {
    if (Distances[i] < 10)
      Data += "00" + String((int)Distances[i]);
    else if (Distances[i] < 100)
      Data += "0" + String((int)Distances[i]);
    else
      Data += String((int)Distances[i]);
  }

  Serial.println(Data); // 현재 상태 출력(PC(Python)로 전송)
}

//------------------- Control--------------------

// 차량 방향 -> 차량 속도 -> 초음파 데이터 수집 -> 굴절각 데이터 수집 -> 조향 제어
void Car_Control()
{
  if (Receive_Direction == 0) // 구동 방향 전진으로 제어
  {
    digitalWrite(Direction_Pin, LOW);
    Direction = 0;
  }
  else if (Receive_Direction == 1) // 구동 방향 후진으로 제어
  {
    digitalWrite(Direction_Pin, HIGH);
    Direction = 1;
  }

  Sonic_Control();        // 초음파 데이터 수집
  Middle_Angle_Control(); // 굴절 데이터 수집

  analogWrite(Speed_PWM_Pin, Receive_Speed); // 구동 속도 제어
  Speed = Receive_Speed;

  Steering_Control(); // 조향 제어
}

// 전륜, 후륜 조향각 제어
void Steering_Control()
{
  int F_Angle = abs(Front_Angle - Receive_Front_Angle); // 전륜 조향 변화량 변수
  int R_Angle = abs(Rear_Angle - Receive_Rear_Angle);   // 후륜 조향 변화량 변수

  if (Receive_Front_Angle < Front_Angle) // 수신한 전륜 조향각 < 현재 전륜 조향각  - 전륜 좌회전
  {
    digitalWrite(Front_Steering_DIR_Pin, HIGH);
    Front_Angle--;
  }
  else if (Receive_Front_Angle > Front_Angle) // 수신한 전륜 조향각 > 현재 전륜 조향각 - 전륜 우회전
  {
    digitalWrite(Front_Steering_DIR_Pin, LOW);
    Front_Angle++;
  }

  if (Receive_Rear_Angle < Rear_Angle) // 수신한 후륜 조향각 < 현재 후륜 조향각 - 후륜 좌회전
  {
    digitalWrite(Rear_Steering_DIR_Pin, HIGH);
    Rear_Angle--;
  }
  else if (Receive_Rear_Angle > Rear_Angle) // 수신한 후륜 조향각 > 현재 후륜 조향각 - 후륜 우회전
  {
    digitalWrite(Rear_Steering_DIR_Pin, LOW);
    Rear_Angle++;
  }

  if (F_Angle != 0 && R_Angle != 0)
  {
    analogWrite(Front_Steering_PWM_Pin, Front_Steering_Speed);
    analogWrite(Rear_Steering_PWM_Pin, Rear_Steering_Speed);
    delay(Angle_delay_front);
    analogWrite(Front_Steering_PWM_Pin, 0);
    delay(Angle_delay_rear - Angle_delay_front);
    analogWrite(Rear_Steering_PWM_Pin, 0);
    delay(Angle_delay_term);
  }
  else if (F_Angle != 0 && R_Angle == 0)
  {
    analogWrite(Front_Steering_PWM_Pin, Front_Steering_Speed);
    analogWrite(Rear_Steering_PWM_Pin, 0);
    delay(Angle_delay_front);
    analogWrite(Front_Steering_PWM_Pin, 0);
    analogWrite(Rear_Steering_PWM_Pin, 0);
    delay(Angle_delay_term);
  }
  else if (F_Angle == 0 && R_Angle != 0)
  {
    analogWrite(Front_Steering_PWM_Pin, 0);
    analogWrite(Rear_Steering_PWM_Pin, Rear_Steering_Speed);
    delay(Angle_delay_rear);
    analogWrite(Front_Steering_PWM_Pin, 0);
    analogWrite(Rear_Steering_PWM_Pin, 0);
    delay(Angle_delay_term);
  }
  else
  {
    analogWrite(Front_Steering_PWM_Pin, 0);
    analogWrite(Rear_Steering_PWM_Pin, 0);
  }
}

// 초음파 센서 제어
void Sonic_Control()
{
  Get_Distance(); // 초음파 데이터 수집

  for (int i = 0; i < S_Sensor_Num; i++)
  {
    if (Distances[0] <= 50.0) // 각 센서 50cm이하일 경우 정지
      Receive_Speed = 0;
  }
}

void Get_Distance()
{
  double duration = 0, distance = 0; // 데이터 인식 시간 임시 저장, 수신한 데이터 거리

  int Num = ECHO_F + Sonic_Count; // 1~6 초음파 순서대로 인식하기 위한 변수
  digitalWrite(TRIG, LOW);        // 트리거 LOW 초기화
  digitalWrite(Num, LOW);         // 에코 LOW 초기화
  delayMicroseconds(2);           // 안정화 대기
  digitalWrite(TRIG, HIGH);       // 트리거 HIGH 발사 시작
  delayMicroseconds(10);          // 트리거 연속 발사 대기
  digitalWrite(TRIG, LOW);        // 트리거 LOW 발사 종료

  duration = pulseIn(Num, HIGH, 250 * 58.2); // 25~200cm까지 인식 시간
  distance = duration / 58.2;                // 거리 계산

  if (distance != 0) // 25~200cm 일 경우
  {
    Distances[Sonic_Count] = distance; // 현재 순서의 초음파 거리 저장
    Distances_Chance[Sonic_Count] = 0; // 필터 초기화
  }
  else // 25~200cm가 아닌 이외의 경우
  {
    Distances_Chance[Sonic_Count] += 1;      // 필터(오류) 증가
    if (Distances_Chance[Sonic_Count] >= 20) // 필터(오류) 20개 이상일 경우
    {
      Distances[Sonic_Count] = 250;       // 현재 순서의 초음파 200cm로 저장
      Distances_Chance[Sonic_Count] = 20; // 필터(오류) 20으로 초기화
    }
  }

  Sonic_Count++;                   // 다음 순서의 초음파를 위한 변수 증가
  if (Sonic_Count == S_Sensor_Num) // 처음 초음파로 순서 제어
  {
    Sonic_Count = 0;
  }
}

// 굴절각 초기화
// Example)
// 현재 210 수신 -> Middle_Angles[210] = 180 으로 저장 -> 이어서 Middle_Angles[209] = 179..... 이런식으로 저장
// 응용 방식 -> 현재 굴절각 230 입력시 -> Middle_Angles[230] = 200 이므로 현재 굴절각은 200으로 전송
void Middle_Angle_init()
{
  int input_middle_angle;
  int temp_angle = 180; // 180으로 현재의 굴절을 인식
  int m_sum = 0;
  int m_count = 0;
  while (true)
  {
    input_middle_angle = analogRead(Middle_Encoder_Pin); // 현재 굴절각 수집
    if (0 < input_middle_angle && input_middle_angle < 360 && m_count < 50)
    {
      m_sum += input_middle_angle;
      m_count++;
    }
    if (m_count == 50)
    {
      input_middle_angle = m_sum / m_count;
      break;
    }
  }

  for (int i = input_middle_angle; i >= 0; i--) // 현재 굴절각을 기준으로 왼쪽 굴절각 초기화
  {
    if (temp_angle >= 0)
    {
      Middle_Angles[i] = temp_angle;
    }
    else
    {
      Middle_Angles[i] = 360 + temp_angle;
    }
    temp_angle--;
  }
  temp_angle = 180;
  for (int i = input_middle_angle; i < 360; i++) // 현재 굴절각을 기준으로 오른쪽 굴절각 초기화
  {
    if (temp_angle < 360)
    {
      Middle_Angles[i] = temp_angle;
    }
    else
    {
      Middle_Angles[i] = temp_angle - 360;
    }
    temp_angle++;
  }
}

// 현재 수신되는 굴절각 제어
void Middle_Angle_Control()
{
  int input_middle_angle = analogRead(Middle_Encoder_Pin); // 굴절각 데이터 수집
  if (0 < input_middle_angle && input_middle_angle < 360)
    Middle_Angle = 360 - Middle_Angles[input_middle_angle]; // 초기화된 굴절각으로 데이터 변환 작업 후 현재 굴절각 저장
}

void setup()
{
  Serial.begin(115200);

  // Engine Motor 구동 모터 초기화
  analogWrite(Speed_PWM_Pin, 0);
  pinMode(Direction_Pin, OUTPUT);
  pinMode(Speed_PWM_Pin, OUTPUT);
  analogWrite(Speed_PWM_Pin, 0);

  // Front Steering Motor 전륜 모터 초기화

  analogWrite(Front_Steering_PWM_Pin, 0);
  pinMode(Front_Steering_DIR_Pin, OUTPUT);
  pinMode(Front_Steering_PWM_Pin, OUTPUT);
  analogWrite(Front_Steering_PWM_Pin, 0);

  // Rear Steering Motor 후륜 모터 초기화
  analogWrite(Rear_Steering_PWM_Pin, 0);
  pinMode(Rear_Steering_DIR_Pin, OUTPUT);
  pinMode(Rear_Steering_PWM_Pin, OUTPUT);
  analogWrite(Rear_Steering_PWM_Pin, 0);

  // Microsonic Sensor 초음파 센서 초기화
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO_F, INPUT);
  pinMode(ECHO_R, INPUT);
  pinMode(ECHO_A_L, INPUT);
  pinMode(ECHO_A_R, INPUT);
  pinMode(ECHO_B_L, INPUT);
  pinMode(ECHO_B_R, INPUT);

  // Middle_Encoder_Pin
  Middle_Angle_init();    // 굴절각 초기화
  Middle_Angle_Control(); // 굴절각 제어
}

void loop()
{
  Serial_Receive(); // PC(Python) -> Arduino 데이터 수신
  //camera_on = true;

  if (!camera_on)
  {
    Serial_Start(); // PC(Python) 통신 연결 시도 및 확인
  }
  else
  {
    Serial_Process(); // PC(Python) -> Arduino 수신된 데이터 변환
    Car_Control();    // 수신한 데이터를 통해 제어
    Serial_Send();    // Arduino -> PC(Python) 제어된 현재 데이터 전송
  }
}
