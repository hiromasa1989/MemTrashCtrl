#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
#include <algorithm>
#include <unistd.h>

#define PI 3.1415926535
#define SCENE 24

//角度からラジアンに変換します
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )   

class MyController : public Controller {  
public:  
  void onInit(InitEvent &evt);  
  double onAction(ActionEvent&);  
  void onRecvMsg(RecvMsgEvent &evt); 
  void onCollision(CollisionEvent &evt); 

  /* @brief  位置を指定しその方向に回転を開始し、回転終了時間を返します
   * @param  pos 回転したい方向の位置
   * @param  vel 回転速度
   * @param  now 現在時間
   * @return 回転終了時間
   */
  double rotateTowardObj(Vector3d pos, double vel, double now);
  

  /* @brief  位置を指定しその方向に進みます
   * @param  pos   行きたい場所
   * @param  vel   移動速度
   * @param  range 半径range以内まで移動
   * @param  now   現在時間
   * @return 到着時間
   */
  double goToObj(Vector3d pos, double vel, double range, double now);

private:
  RobotObj *m_my;

  // ゴミの場所
  Vector3d m_tpos;  

  // ゴミの名前
  std::string m_tname;  

  /* ロボットの状態
   * 0 初期状態
   * 1 サービスからの認識結果を待っている状態
   * 2 ゴミがある方向に回転している状態
   * 3 関節を曲げてゴミを取りに行っている状態
   * 4 ゴミを持ってゴミ箱の方向に回転している状態
   * 5 ゴミを持ってゴミ箱に向かっている状態
   * 6 ゴミを捨てて関節角度を元に戻している状態
   * 7 元に場所に戻る方向に回転している状態
   * 8 元の場所に向かっている状態
   * 9 元の向きに回転している状態
   */
  int m_state; 

  // 車輪の角速度
  double m_vel;

  // 関節の回転速度
  double m_jvel;

  // 車輪半径
  double m_radius;

  // 車輪間距離
  double m_distance;

  // 移動終了時間
  double m_time;

  // 初期位置
  Vector3d m_inipos;

  // grasp中かどうか
  bool m_grasp;

  // ゴミ認識サービス
  BaseService *m_recogSrv;
  ViewService *m_view;

  // サービスへのリクエスト複数回送信を防ぐ
  bool m_sended;

  int dx, dy, dz;
  int vx, vy, vz;

  Vector3d vec;
  Vector3d ini_vec;
  Rotation rot;

  SimObj *obj;

  int theta;//Direction of the sun from original point

};  

void MyController::onInit(InitEvent &evt) 
{  
  m_my = getRobotObj(myname());

  // 初期位置取得
  m_my->getPosition(m_inipos);

  // 物体の位置方向初期化
  dx = 0;
  dy = 0;
  dz = 0;
  vx = 0;
  vy = 0;
  vz = 0;

  // 太陽光の初期化
  theta = 0;

  // grasp初期化
  m_grasp = false;
  m_recogSrv = NULL;
  m_sended = false;
}  
  
double MyController::onAction(ActionEvent &evt)
{  
  switch(m_state){

  // 初期状態
  case 0: {
    if(m_recogSrv == NULL){
      // 物体記憶サービスが利用可能か調べる
      if(checkService("MemObj")){
        LOG_MSG(("Step0"));
	// 物体記憶サービスに接続
	m_recogSrv = connectToService("MemObj");
        m_state = 1;
        // 物体設定
        //obj = getObj("petbottle_1");
        obj = getObj("petbottle_2");
        //obj = getObj("petbottle_3");
        //obj = getObj("can_1");
        //obj = getObj("can_2");
        //obj = getObj("mugcup_1");
	//obj = getObj("mugcup_2");
	obj->getPosition(ini_vec);
        // 物体除外
        obj->setPosition(0, 0, 0); 
        broadcastMsgToSrv("Memorizing scene");
      }
    }
    break;
  }

  case 1: {
    if(m_recogSrv != NULL && m_sended == false){
      LOG_MSG(("Step1"));
     // 背景記憶リクエスト送信
      char msg[1024];
      sprintf(msg, "MemRef"); 
      m_recogSrv->sendMsgToSrv(msg);
      m_sended = true;
    }
    break;
  }

  case 2: {
    LOG_MSG(("Step2"));
    // 物体初期位置設定
    obj->setPosition(ini_vec.x(), ini_vec.y(), ini_vec.z()); 
    broadcastMsgToSrv("Memorizing object");
    m_state = 3;
    break;
  }

  case 3: {
    if(m_recogSrv != NULL && m_sended == false){
      LOG_MSG(("Step3"));
     // 物体記憶リクエスト送信
      char msg[1024];
      sprintf(msg, "MemInp"); 
      m_recogSrv->sendMsgToSrv(msg);
      m_sended = true;
    }
    break;
  }

  case 4: {
    if(m_recogSrv != NULL && m_sended == false){
      LOG_MSG(("Step4"));
     // 物体領域リクエスト送信
      char msg[1024];
      sprintf(msg, "ExtArea"); 
      m_recogSrv->sendMsgToSrv(msg);
      m_sended = true;
    }
    break;
  }

  case 5: {
    if(m_recogSrv != NULL && m_sended == false){
      LOG_MSG(("Step5"));
     // 影設定リクエスト送信
      char msg[1024];
      sprintf(msg, "SetShadow"); 
      m_recogSrv->sendMsgToSrv(msg);
      m_sended = true;
    }
    break;
  }

  case 6: {
    if(m_recogSrv != NULL && m_sended == false){
      LOG_MSG(("Step6"));
       // 照明設定リクエスト送信
      char msg[1024];
      sprintf(msg, "SetLight"); 
      m_recogSrv->sendMsgToSrv(msg);
      m_sended = true;
    }
    break;
  }

  case 7: {
    if(m_recogSrv != NULL && m_sended == false){
      LOG_MSG(("Step7"));
     // 物体記憶リクエスト送信
      char msg[1024];
      sprintf(msg, "MemObject"); 
      m_recogSrv->sendMsgToSrv(msg);
      m_sended = true;
    }
    break;
  }

  case 8: {
    LOG_MSG(("Step8"));     
    // 照明設定変更
    if(theta == SCENE){
        theta = 0;
        m_state = 9;
    }
    else{
	theta ++;
    	m_state = 6;
    }
    break;
  }

  case 9: {
    broadcastMsgToSrv("Finished");
    break;
  }

  }
  return 0.1;      
}  
  
void MyController::onRecvMsg(RecvMsgEvent &evt)
{  
  // 送信者取得
  std::string sender = evt.getSender();

  // 送信者が物体記憶サービスの場合
  if(sender == "MemObj"){
    char *all_msg = (char*)evt.getMsg();
    m_sended = false;   
    // 次の姿勢に移動
    m_state++;
  }
}  

void MyController::onCollision(CollisionEvent &evt) 
{

}
  
double MyController::rotateTowardObj(Vector3d pos, double velocity, double now)
{

}

// object まで移動
double MyController::goToObj(Vector3d pos, double velocity, double range, double now)
{

}

extern "C" Controller * createController() {  
  return new MyController;  
}  

