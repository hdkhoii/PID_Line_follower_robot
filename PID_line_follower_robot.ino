// Khai báo các chân L298N (Module điều khiển động cơ)
#define Int1 7
#define Int2 6
#define EnR 3 // Điều khiển động cơ trái
#define Int3 5
#define Int4 4
#define EnL 9 // Điều khiển động cơ phải
# define base_speed 65
// Cài đặt vận tốc động cơ
#define max_speed 110 // Tốc độ tối đa

int sensor[5]={0,0,0,0,0};
double IRvalue = 0.0,pre_IRvalue = 0.0,sum=0.0;
float last_IRvalue = 0.0;
float P=0.0,I=0.0,D=0.0;
double output=0.0;
int left_speed;
int right_speed;
const float Kp = 1;
const float Ki = 0.01;
const float Kd = 1;


void setup() {
  Serial.begin(9600);

  // Cài đặt chế độ đầu ra cho động cơ
  pinMode(EnR, OUTPUT);
  pinMode(Int1, OUTPUT);
  pinMode(Int2, OUTPUT);
  pinMode(EnL, OUTPUT);
  pinMode(Int3, OUTPUT);
  pinMode(Int4, OUTPUT);

  // Cài đặt chế độ đầu vào cho cảm biến
  pinMode(11, INPUT);
  pinMode(13, INPUT);
  pinMode(12, INPUT);
  pinMode(8, INPUT);
  pinMode(10, INPUT);
}
unsigned long start =0;


void loop() {

unsigned long now = micros();
unsigned long duration = now - start;
  float dt = (float) duration / 1000000.0;  // dt tính bằng giây
  start = now;
  ProcessIR();
  PID_func(dt);
  Motor_control();
  delay(10);
//  Serial.print("PID Output (raw): ");
//  Serial.println(output);
}

void ProcessIR(){
  // Đọc cảm biến và đảo ngược giá trị: vạch đen (1), trắng (0)
  sensor[0] = digitalRead(8);
  sensor[1] = digitalRead(10);
  sensor[2] = digitalRead(11);
  sensor[3] = digitalRead(12);
  sensor[4] = digitalRead(13);

  IRvalue = 0;

  // Các TH hợp lý khi chỉ có 1 vạch đen
  if (sensor[0]==1 && sensor[1]==0 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0) {
    IRvalue = -4.0;
  }
  else if (sensor[0]==0 && sensor[1]==1 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0) {
    IRvalue = -2.0;
  }
  else if (sensor[0]==0 && sensor[1]==0 && sensor[2]==1 && sensor[3]==0 && sensor[4]==0) {
    IRvalue = 0.0;
  }
  else if (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==1 && sensor[4]==0) {
    IRvalue = 2.0;
  }
  else if (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==0 && sensor[4]==1) {
    IRvalue = 4.0;
  }
  // Trường hợp chuyển tiếp 2 sensor thấy line
  else if (sensor[0]==1 && sensor[1]==1 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0) {
    IRvalue = -3.0;
  }
  else if (sensor[0]==0 && sensor[1]==1 && sensor[2]==1 && sensor[3]==0 && sensor[4]==0) {
    IRvalue = -1.0;
  }
  else if (sensor[0]==0 && sensor[1]==0 && sensor[2]==1 && sensor[3]==1 && sensor[4]==0) {
    IRvalue = 1.0;
  }
  else if (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==1 && sensor[4]==1) {
    IRvalue = 3.0;
  }

  // Mất line: tất cả đều trắng
  else if (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0) {
    IRvalue = last_IRvalue; // Giữ hướng cũ
  }

  // Lưu hướng gần nhất nếu không mất line
  if (!(sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0)) {
    last_IRvalue = IRvalue;
  }
}


void PID_func(float dt) {
  float error = (float)IRvalue;

  P = error;
  I += error * dt;
  D = (error - pre_IRvalue) / dt;
  pre_IRvalue = error;
  output = Kp * P + Ki * I + Kd * D;  
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Điều khiển động cơ
void Motor_control() {
  float offset = mapFloat(output, -4, 4, -100, 100);  // Dùng offset từ PID
  offset = constrain(offset, -50, 50);                // Giới hạn lệch tốc độ

  // Xử lý trường hợp lệch nhẹ (bình thường)
  if (abs(IRvalue) < 3) {
    left_speed = constrain(base_speed - offset, 0, max_speed);
    right_speed = constrain(base_speed + offset, 0, max_speed);
  }
  // Lệch lớn → tăng cường phản hồi nhưng vẫn không đổi chiều
  else {
    float boosted_offset = offset * 1;
    boosted_offset = constrain(boosted_offset, -90, 90);

    left_speed = constrain(base_speed - boosted_offset, 0, max_speed);
    right_speed = constrain(base_speed + boosted_offset, 0, max_speed);
  }

  // Luôn chạy tiến
  digitalWrite(Int1, HIGH);  // Trái tiến
  digitalWrite(Int2, LOW);
  digitalWrite(Int3, LOW);   // Phải tiến
  digitalWrite(Int4, HIGH);

  analogWrite(EnL, left_speed);
  analogWrite(EnR, right_speed);
}
