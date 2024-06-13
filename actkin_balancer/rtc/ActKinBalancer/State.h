#ifndef ACTKINBALANCER_STATE_H
#define ACTKINBALANCER_STATE_H

#include <contact_state_msgs/idl/ContactState.hh>
#include <unordered_map>
#include <memory>
#include <cnoid/Body>

namespace actkin_balancer{
  class Region3D {
  public:
    Eigen::MatrixXd C = Eigen::MatrixXd::Identity(3,3);
    cnoid::VectorX ld = cnoid::VectorX::Zero(3);
    cnoid::VectorX ud = cnoid::VectorX::Zero(3);
  };

  class Contact {
  public:
    cnoid::LinkPtr link1;
    cnoid::Isometry3 localPose1 = cnoid::Isometry3::Identity();
    cnoid::LinkPtr link2;
    bool freeX = false;
    bool freeY = false;
    //std::shared_ptr<ik_constraint2::PositionConstraint> ikc;
  };

  // このクラスの中身は、動作中は変更されない
  class EEParam {
  public:
    std::string name = "rleg";
    cnoid::LinkPtr parentLink = nullptr;
    cnoid::Isometry3 localPose = cnoid::Isometry3::Identity(); // Parent Link Frame
    cnoid::Vector3 copOffset = cnoid::Vector3{0.0,0.02,0.0};
    std::vector<cnoid::Vector3> hull = std::vector<cnoid::Vector3>{cnoid::Vector3(0.115,0.065,0.0),cnoid::Vector3(-0.095,0.065,0.0),cnoid::Vector3(-0.095,-0.065,0.0),cnoid::Vector3(0.115,-0.065,0.0)}; // endeffector frame.  凸形状で,上から見て半時計回り. Z成分は0. 単位[m]. 干渉計算に使用される. JAXONでは、COPがX -0.1近くにくるとギア飛びしやすいので、少しXの下限を少なくしている.
    std::vector<cnoid::Vector3> safeHull = std::vector<cnoid::Vector3>{cnoid::Vector3(0.075,0.055,0.0),cnoid::Vector3(-0.075,0.055,0.0),cnoid::Vector3(-0.075,-0.055,0.0),cnoid::Vector3(0.075,-0.055,0.0)}; // endeffector frame. 単位[m]. 凸形状で,上から見て半時計回り. Z成分は0でなければならない. 大きさはhull以下

    // stride parameters
    double defaultStepTime = 0.8; // [s]. defaultのfootstepの一歩の時間. 0より大きい
    double defaultDoubleSupportRatio = 0.15; // defaultStepTimeのうちの、両足支持期の時間の割合. 0より大きく1未満
    cnoid::Vector3 defaultTranslatePos = cnoid::Vector3{0.0,-0.1,0.0}; // 右脚と左脚の中心からの右脚の相対位置.([m]). Z座標は0でなければならない.
    std::vector<cnoid::Vector3> defaultStrideLimitationHull = std::vector<cnoid::Vector3>{cnoid::Vector3(0.15,-0.18,0),cnoid::Vector3(-0.15,-0.18,0),cnoid::Vector3(-0.15,-0.35,0),cnoid::Vector3(0.15,-0.35,0)}; // 単位[m]. defaultのfootstepの、遊脚のエンドエフェクタの着地位置の範囲の凸包. 反対の脚のEndEffector frame(Z軸は鉛直)で表現した着地可能領域(自己干渉やIKの考慮が含まれる). Z成分は0でないといけない. 凸形状で,上から見て半時計回り. thetaの影響はlegHullとlegCollisionMarginを用いて別で評価されるので、defaultStrideLimitationHullでは考慮しなくて良い. 左右方向にsteppable regionをまたぐ場合は、これのY成分が大きくないと後ろ足がまたげない
    double defaultStepHeight = 0.05; // defaultのfootstepの足上げ高さ[m]. 0以上
    double defaultTouchVel = 0.3; // 0より大きい. 単位[m/s]. この速さで足を下ろした場合に着地までに要する時間をremainTimeが下回るまで、足下げを始めない. 0.5だと実機では少し速すぎるか?

    double maxSwingXYVelocity = 1.0; // 0より大きい. 単位[m/s].
    double maxSwingLandVelocity = 0.5; // 0より大きい[m/s]. touchVelと同じくらい?
    double maxSwingLiftVelocity = 0.14; // 0より大きい[m/s]. 0.2m登るときに一歩あたり1.4sくらい?
    double strideLimitationMaxTheta = 0.261799; // footstepの旋回上限. 支持脚相対. default 15[deg]. 0以上. 足裏同士の干渉は自動で回避できるが、膝同士の干渉はIK以外では回避できないので、内股方向には小さくすること.
    double strideLimitationMinTheta = -0.785398; // footstepの下限. 支持脚相対. default -45[deg]. 0以下. 足裏同士の干渉は自動で回避できるが、膝同士の干渉はIK以外では回避できないので、内股方向には小さくすること.
    std::vector<cnoid::Vector3> strideLimitationHull = std::vector<cnoid::Vector3>{cnoid::Vector3(0.35,-0.15,0),cnoid::Vector3(-0.35,-0.15,0),cnoid::Vector3(-0.35,-0.350,0),cnoid::Vector3(-0.20,-0.45,0),cnoid::Vector3(0.20,-0.45,0),cnoid::Vector3(0.35,-0.350,0)}; // footstepの上下限の凸包. 反対の脚のEndEffector frame(Z軸は鉛直)で表現した着地可能領域(自己干渉やIKの考慮が含まれる). Z成分は0でないといけない. 凸形状で,上から見て半時計回り. thetaの影響はhullとcollisionMarginを用いて別で評価されるので、strideLimitationHullでは考慮しなくて良い. 斜め方向の角を削るなどして、IKが解けるようにせよ.
    double collisionMargin = 0.02; // [m]. 左右の足のhullがこの距離以上離れるようにする. 0以上.
    double maxLandingHeight = 0.25; // [m]. 反対の脚のEndEffector frame(Z軸は鉛直)で表現した着地高さの上限(自己干渉やIKの考慮が含まれる).
    double minLandingHeight = -0.25; // [m]. 反対の脚のEndEffector frame(Z軸は鉛直)で表現した着地高さの下限(自己干渉やIKの考慮が含まれる). overwritableMaxLandingHeight以下.
    double goalOffset = -0.02; // [m]. 遊脚軌道生成時に、次に着地する場合、鉛直方向に, 目標着地位置に対して加えるオフセット. 0以下. 遅づきに対応するためのもの. 位置制御だと着地の衝撃が大きいので0がよいが、トルク制御時や、高低差がある地形や、衝撃を気にする必要がないシミュレーションでは-0.05等にした方がよい.




    // 出力用
    double muTrans = 0.5; // 大股で歩くときはZMP-COMの位置関係的に、垂直抗力と並進力の比がそこまで大きな差にならないので、0.1~0.3程度だと足りない場合がある.
    double muRot = 0.05; // 旋回歩行時に必要なので、小さすぎてはいけない
    double regionMargin = 0.05; // legHullの周囲[m]をregionとする

    Eigen::MatrixXd wrenchC; // endeffector frame. link1がlink2から受ける力に関する接触力制約. ?x6
    cnoid::VectorX wrenchld;
    cnoid::VectorX wrenchud;
    Region3D region; // endeffector frame.

    void flipY(EEParam& param); //このEEParamのY成分を反転させてparamにコピーする. nameとparentLink以外
  };



  // このクラスのメンバ変数は、全てfiniteである(nanやinfが無い)ことが仮定されている. 外部から値をセットするときには、finiteでない値を入れないようにすること
  class State {
  public:
    // from data port. 狭義のstate
    cnoid::BodyPtr robot; // actual.
    cnoid::Vector3 cogVel = cnoid::Vector3::Zero();
    std::vector<std::shared_ptr<Contact> > contacts; // actual

    // objects
  public:
    std::unordered_map<std::string, cnoid::LinkPtr> linkNameMap; // MODE_ABC中はconstant. URDFのLink名 -> linkPtr

  public:
    // parameters. contactable pointの情報.
    std::vector<EEParam> ee = std::vector<EEParam>(2); // rleg, lleg

  public:

    // RTC起動時に一回呼ばれる.
    void init(const cnoid::BodyPtr& robot_);

    // startBalancer時に呼ばれる
    void onStartBalancer();

    // MODE_ST中のみ呼ばれる
    void updateRobotFromIdl(const RTC::TimedDoubleSeq& m_qAct, const RTC::TimedDoubleSeq& m_dqAct, const RTC::TimedPose3D& m_actBasePose, const RTC::TimedVelocity3D& m_actBaseVel, double dt);
    void updateContactFromIdl(const contact_state_msgs::TimedContactSeq& m_actContactState);
  };

};

#endif
