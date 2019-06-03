/***时间戳：2019/6/3 18:20创建***/
/***更新：删除一些无用注释***/

#include <Servo.h>//之间要有空格，否则编译时会报错

/*宏定义*/
#define uchar unsigned char//关键字宏定义
#define MIN 45
#define MAX 135
#define DEL 10//10或20
#define N 10 //注：N = (MAX-MIN)/DEL+1;19或10
#define PASSING_THRESH 1 //通行阈值
#define ULTRA_DISTANCE 15//超声波阈值(cm)


/*定义结构体*/
typedef struct
{
  float Kp, Ki, Kd;
  float p_out, i_out, d_out;
//  int present, target;
  float error, last_error, last_error2, last_error3 , last_error4, last_error5;
  int output;
}PID_TypeDef;

/*定义结构体变量*/
PID_TypeDef myPID, PID_y;//PID结构体myPID

/*定义函数*/
void create_map(int);
int find_path(void);
void DriveMortor(uchar,uchar,uchar, uchar);//双电机驱动函数；dir代表方向，0为停止，1为正转，2为反转；sp代表转速，PWM调速范围为0-255

int pid(int present, int target, PID_TypeDef* PID);//三个参数：当前位置，目标位置，PID结构体
void pid_init(float Kp, float Ki, float Kd, PID_TypeDef* PID);//4个参数：Kp,Ki,Kd,PID结构体

/*定义变量*/
int deg;//角度
int tarAna[N];//目标(模拟)数组
int maxtar=0;//最大目标(模拟)
int maxtarInd=0;//最大目标(模拟)下标
int tar[N];//目标数组
int obs[N];//障碍数组
int plan[N];//计划数组
int direct = 1;//扫描方向
int k=0;//计数变量(数组元素序号)
int infra_intens = 0;//红外读数?
int light_intens = 0;//光敏读数?

int path = -1;//路 默认无路可走
uchar highFlag = 0;//高电平标志
int cnt[2];//计数变量(历史最大宽度、当前宽度)

int stopFlag=0;

float middle;//中轴
int speedA;
int speedB;
int speedM=80;//平均速度(最大191)
int speedDiff=0;//1/2速度差

//超声波
const int TrigPin = 11;//设定SR04连接的Arduino引脚
const int EchoPin = 13;
unsigned int distance;

/*定义引脚*/
uchar infraPin = 12;
uchar lightPin = 0;
uchar servoPin = 9;

uchar EAmortorA=3;//定义数字接口3为电机A使能端
uchar IN1mortorA=2;//定义数字接口2为电机A方向端
uchar IN2mortorA=4;//定义数字接口4为电机A方向端
uchar EAmortorB=6;//定义数字接口6为电机B使能端
uchar IN3mortorB=7;//定义数字接口7为电机B方向端
uchar IN4mortorB=8;//定义数字接口8为电机B方向端


Servo myservo;//定义舵机变量名

void setup()
{
  //超声波
  pinMode(TrigPin, OUTPUT);//初始化连接SR04的引脚
  pinMode(EchoPin, INPUT);
  
  pinMode(EAmortorA,OUTPUT);//定义数字接口3为输出
  pinMode(IN1mortorA,OUTPUT);//定义数字接口2为输出
  pinMode(IN2mortorA,OUTPUT);//定义数字接口4为输出
  pinMode(EAmortorB,OUTPUT);//定义数字接口6为输出
  pinMode(IN3mortorB,OUTPUT);//定义数字接口7为输出
  pinMode(IN4mortorB,OUTPUT);//定义数字接口8为输出
  
//  Serial.begin(9600);//连接到串行端口，波特率为9600(调试用)
  myservo.attach(servoPin);//定义数字9为舵机接口
  pinMode(infraPin, INPUT); //设定infraPin引脚为输入状态
  pinMode(lightPin, INPUT); //设定lightPin引脚为输入状态
  pid_init(15, 0, 0, &myPID);//初始化pid参数
  middle = N/2;
  

  deg = MIN; //初始化舵机角度为45度
  myservo.write(deg);
  delay(500);//初始给他时间转
  
  while(k!=0 || direct!=-1)
  {
//    /*+++++++++++++调试用+++++++++++*/
//    Serial.print("k: ");
//    Serial.print(k,DEC);//显示val变量的值
//    Serial.print(" direct: ");
//    Serial.println(direct,DEC);//显示val变量的值
    /*-------------调试用-----------*/
    
    myservo.write(deg);
    delay(100);//给他时间转
    
    /*+++++++++++++调试用+++++++++++*/
//    Serial.print("Current degree: ");
//    Serial.println(deg,DEC);//显示当前角度
    /*-------------调试用-----------*/
    
    obs[k] = digitalRead(infraPin); //当前红外传感器值传入障碍数组
    tarAna[k] = analogRead(lightPin); //当前光敏传感器值传入目标(模拟)数组
    tar[k]=0;
//    Serial.println(tarAna[k]);//
//    Serial.println(maxtar);//
    if(tarAna[k]>maxtar)
    {
      maxtarInd = k;
      maxtar = tarAna[k];
//      Serial.println(">\n");//
    }
    
//    /*+++++++++++++调试用+++++++++++*/
//    Serial.println("\nObstacle map: ");
//    for(int i=0; i<N; i++)
//    {
//      Serial.print(obs[i],DEC);//显示val变量的值
//    }
//    Serial.println("\nTarget map: ");
//    for(int i=0; i<N; i++)
//    {
//      Serial.print(tarAna[i],DEC);//显示val变量的值
//    }
//    /*-------------调试用-----------*/
    
    if(k==N-1 && direct==1)
    {
//      Serial.println("!!!!!");//显示val变量的值
//      Serial.println(k,DEC);//显示val变量的值
//      Serial.println(N,DEC);//显示val变量的值
      direct = -1;
      
//      Serial.println("\n找虫子第一回: ");
//      for(int i=0; i<N; i++)
//      {
//        Serial.print(tar[i]);//显示目标图
//        Serial.print(" ");//
//      }
      
      for(int i=-2; i<=2; i++)
      {
        if((maxtarInd+i)>=0 && (maxtarInd+i)<=N-1)
        {
          tar[maxtarInd+i]=1;//maxtarInd前后5个置1
        }
      }

//      Serial.println("\n找虫子第1.5回: ");
//      Serial.println(maxtar);//
//      Serial.println(maxtarInd);//
//      Serial.println();//
//      for(int i=0; i<N; i++)
//      {
//        Serial.print(tar[i]);//显示目标图
//        Serial.print(" ");//
//      }
      maxtar = 0;//复位
      k = k - direct;//修正错误！(反方向走时重复一下)
      deg = deg - direct*DEL;//修正错误！(反方向走时重复一下)
    }

    k = k + direct;//计数值加一或减一
    deg = deg + direct*DEL;//角度加DEL或减DEL
    
  }

//  Serial.println("\n找虫子第二回: ");
//  Serial.println(k);
//  for(int i=0; i<N; i++)
//  {
//    Serial.print(tar[i]);//显示目标图
//    Serial.print(" ");//
//  }
  
  for(int i=-2; i<=2; i++)
  {
    if((maxtarInd+i)>=0 && (maxtarInd+i)<=N-1)
    {
      tar[maxtarInd+i]=1;//maxtarInd前后5个置1
    }
  }
  
  /*+++++++++++++调试用+++++++++++*/
//  Serial.println("\nOriginal Obstacle map: ");
//  for(int i=0; i<N; i++)
//  {
//    Serial.print(obs[i],DEC);//显示障碍图
//    Serial.print(" ");//
//  }
//  Serial.println("\nOriginal Target map: ");
//  for(int i=0; i<N; i++)
//  {
//    Serial.print(tarAna[i]);//显示目标(模拟)图
//    Serial.print(" ");//显示目标(模拟)图
//  }
//  Serial.println("\nOriginal Target map: ");
//  for(int i=0; i<N; i++)
//  {
//    Serial.print(tar[i]);//显示目标图
//    Serial.print(" ");//
//  }
//  /*-------------调试用-----------*/

  k = k - direct;//修正错误！(反方向走时重复一下)
  deg = deg - direct*DEL;//修正错误！(反方向走时重复一下)
}



/**********************************************************
***********************主循环*******************************
***********************************************************/

void loop()
{
  myservo.write(deg);
  delay(100);//给他时间转
  
  /*+++++++++++++调试用+++++++++++*/
//  Serial.print("\nCurrent degree: ");
//  Serial.println(deg,DEC);//显示当前角度
  /*-------------调试用-----------*/
  
  obs[k] = digitalRead(infraPin); //当前红外传感器值传入障碍数组
  tarAna[k] = analogRead(lightPin); //当前光敏传感器值传入目标(模拟)数组
  tar[k]=0;
  if(tarAna[k]>maxtar)
  {
    maxtarInd = k;
    maxtar = tarAna[k];
  }

  //超声波！！
  //产生一个10us的高脉冲去触发TrigPin
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);
  //检测脉冲宽度，并计算出距离（距离(cm)=声波传输时间(us)*0.034(cm/us)/2）
  distance = pulseIn(EchoPin, HIGH)*0.034/2;
//  Serial.print(distance);
//  Serial.print("cm");
//  Serial.println();
  //超声波不能连续发射，要有一定时间间隔
  delay(10);
  
  if((k==N-1 && direct==1)||(k==0 && direct==-1))//扫到最左边/最右边(到头):更新控制参数
  {
    create_map(maxtarInd);//地图、计划图构建

    path = find_path();//寻路

    //决策和控制!!!!!!!!
    if(path==-1 || distance<ULTRA_DISTANCE)//无通路或超声波测到小于15cm
    {
      /*+++++++++++++调试用+++++++++++*/
//      Serial.println("\nNO WAY");
      stopFlag = 1;
      DriveMortor(0,185,0,185);//对于9V电机PWM占空比不能超过191
      delay(1000);//延时1000ms
      DriveMortor(2,80,2,80);//倒车
      delay(1000);//延时1000ms
      DriveMortor(0,185,0,185);//倒车
      delay(1000);//延时1000ms
      /*-------------调试用-----------*/
    }
    else//有通路
    {
      /*+++++++++++++调试用+++++++++++*/
//      Serial.print("\nPath: ");
//      Serial.println(path,DEC);//
//      Serial.print("\nMiddle: ");
//      Serial.println(middle,DEC);//
      /*-------------调试用-----------*/
      
      speedDiff = pid(path,middle,&myPID);
      
      /*+++++++++++++调试用+++++++++++*/
//      Serial.print("\nspeedDiff: ");
//      Serial.println(speedDiff,DEC);//
      /*-------------调试用-----------*/
      
      speedA = speedM+speedDiff;
      speedB = speedM-speedDiff;
      DriveMortor(1,speedA,1,speedB);//对于9V电机PWM占空比不能超过191
    }
    
    

    maxtar = 0;//最大值复位
    direct = -direct;//反向扫
    k = k - direct;//修正错误！(反方向走时重复一下)
    deg = deg - direct*DEL;//修正错误！(反方向走时重复一下)
  }
  
  if(path==-1 || distance<ULTRA_DISTANCE)//无通路或超声波测到小于15cm
  {
    DriveMortor(0,185,0,185);//对于9V电机PWM占空比不能超过191
  }
  else
  {
    DriveMortor(1,speedA,1,speedB);//对于9V电机PWM占空比不能超过191
  }
  
  k += direct;//计数值加一或减一
  deg += direct*DEL;//角度加DEL或减DEL

  
}


//1、绘制地图函数(目标图、计划图)
void create_map(int maxtarInd)
{
  for(int i=-2; i<=2; i++)//求目标数组
  {
    if((maxtarInd+i)>=0 && (maxtarInd+i)<=N-1)
    {
      tar[maxtarInd+i]=1;//maxtarInd前后5个置1
    }
  }

  for(int i=0; i<N; i++)//求计划数组
  {
    plan[i] = tar[i] & obs[i];
  }
  
  /*+++++++++++++调试用+++++++++++*/
//  Serial.println("\n\nObstacle map: ");
//  for(int i=0; i<N; i++)
//  {
//    Serial.print(obs[i],DEC);//显示障碍图
//    Serial.print(" ");//
//  }
  
//    Serial.println("\nTarget(Ana) map: ");
//    for(int i=0; i<N; i++)
//    {
//      Serial.print(tarAna[i]);//显示目标(模拟)图
//      Serial.print(" ");//
//    }
//  
//  Serial.println("\nTarget map: ");
//  for(int i=0; i<N; i++)
//  {
//    Serial.print(tar[i]);//显示目标图
//    Serial.print(" ");//
//  }
//
//  Serial.println("\nPlanning map: ");
//  for(int i=0; i<N; i++)
//  {
//    Serial.print(plan[i]);//显示计划图
//    Serial.print(" ");//
//  }
  /*-------------调试用-----------*/
}


//2、找path函数
int find_path(void)
{
  path = -1;//路 默认无路可走
  highFlag = 0;//高电平标志
  cnt[0] = 0;//计数变量(历史最大宽度)
  cnt[1] = 0;//计数变量(当前宽度)
  /*从左向右遍历plan数组，寻找上升沿和下降沿，寻找最宽的宽度大于PASSING_THRESH的通路*/
  for(int i=0; i<N; i++)
  {
    if(plan[i]>highFlag)//检测到上升沿
    {
      highFlag = 1;//highFlag置1
    }
    
    if(plan[i]<highFlag || (highFlag==1 && i==N-1))//检测到下降沿 或是 到了最右边且为高电平
    {
      highFlag = 0;//highFlag清零
      if(cnt[1]>PASSING_THRESH)//如果当前高电平宽度大于PASSING_THRESH
      {
        if(cnt[1]>cnt[0])//如果当前宽度大于历史最大宽度
        {
          path = int(i - cnt[1]/2);//将高电平中间序号赋值给path
          cnt[0] = cnt[1];//更新历史最大宽度
        }
        cnt[1] = 0;//当前宽度清零
      }
    }
    
    if(highFlag==1)//如果保持高电平
    {
      cnt[1]++;//当前宽度加一
    }
  }
  return path;
}

//3、PID初始化函数，给各参数赋值
//参数(4个)：Kp，Ki，Kd，处理的PID结构体的地址
void pid_init(float Kp, float Ki, float Kd, PID_TypeDef* PID)
{
  PID->Kp = Kp;
  PID->Ki = Ki;
  PID->Kd = Kd;
}

//4、位置PID
//参数(3个)：当前位置，目标位置，处理的PID结构体的地址
int pid(int present, int target, PID_TypeDef* PID)
{
  
  PID->error = target-present;  //本次误差 = 目标值 - 实际值

//  /*+++++++++++++调试用+++++++++++*/
//  Serial.print("\nerror: ");
//  Serial.println(PID->error,DEC);//
  /*-------------调试用-----------*/
  
  PID->p_out = PID->Kp * PID->error;//比例
  PID->i_out += PID->Ki * PID->error;//积分
  PID->d_out = PID->Kd * (PID->error - PID->last_error);//微分
  
  PID->output = PID->p_out + PID->i_out + PID->d_out;//输出
  
  PID->last_error = PID->error;//上次误差 = 本次误差
  
  return PID->output;
}

//5、双电机驱动函数；dir代表方向，0为停止，1为正转，2为反转；sp代表转速，PWM调速范围为0-255
void DriveMortor(uchar dirA,uchar spA,uchar dirB, uchar spB)
{
    switch(dirA)
    {
      case 0://电机停止
        digitalWrite(IN1mortorA,LOW);
        digitalWrite(IN2mortorA,LOW);
        break;
      case 1://电机正转
        digitalWrite(IN1mortorA,HIGH);
        digitalWrite(IN2mortorA,LOW);
        break;
      default://电机反转
        digitalWrite(IN1mortorA,LOW);
        digitalWrite(IN2mortorA,HIGH);
    }

    switch(dirB)
    {
      case 0://电机停止
        digitalWrite(IN3mortorB,LOW);
        digitalWrite(IN4mortorB,LOW);
        break;
      case 1://电机正转
        digitalWrite(IN3mortorB,HIGH);
        digitalWrite(IN4mortorB,LOW);
        break;
      default://电机反转
        digitalWrite(IN3mortorB,LOW);
        digitalWrite(IN4mortorB,HIGH);
    }

    if((spA<=191)&&(spB<=191)) //对于9V电机PWM占空比不能超过191
    {
    analogWrite(EAmortorA,spA);//调节电机A转速(PWM调速范围为0-255)
    analogWrite(EAmortorB,spB);//调节电机B转速(PWM调速范围为0-255)
    }    
}
