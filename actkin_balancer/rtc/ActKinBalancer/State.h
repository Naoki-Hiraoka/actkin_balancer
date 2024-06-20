#ifndef ACTKINBALANCER_STATE_H
#define ACTKINBALANCER_STATE_H

#include <contact_state_msgs/idl/ContactState.hh>
#include <actkin_balancer/idl/ActKinBalancerService.hh>
#include <unordered_map>
#include <memory>
#include <cnoid/Body>

namespace actkin_balancer{
  static const int NUM_LEGS = 2;
  static const int RLEG = 0;
  static const int LLEG = 1;

  class State;

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
    cnoid::Isometry3 localPose = cnoid::Isometry3::Identity(); // Parent Link Frame. クロスできたりジャンプできたりする脚でないと左右方向(外側向き)の着地位置修正は難しいので、その方向に転びそうになることが極力ないように内側にlocalPoseをオフセットさせておくとよい.
    std::vector<Eigen::Vector2d> hull = std::vector<Eigen::Vector2d>{Eigen::Vector2d(0.115,0.065),Eigen::Vector2d(-0.095,0.065),Eigen::Vector2d(-0.095,-0.065),Eigen::Vector2d(0.115,-0.065)}; // endeffector frame.  凸形状で,上から見て半時計回り. 単位[m]. 干渉計算に使用される. JAXONでは、COPがX -0.1近くにくるとギア飛びしやすいので、少しXの下限を少なくしている.  3点以上必要.
    std::vector<Eigen::Vector2d> safeHull = std::vector<Eigen::Vector2d>{Eigen::Vector2d(0.075,0.055),Eigen::Vector2d(-0.075,0.055),Eigen::Vector2d(-0.075,-0.055),Eigen::Vector2d(0.075,-0.055)}; // endeffector frame. 単位[m]. 凸形状で,上から見て半時計回り. 大きさはhull以下

    // stride parameters
    cnoid::Vector3 defaultTranslatePos = cnoid::Vector3{0.0,-0.1,0.0}; // 右脚と左脚の中心からの右脚の相対位置.([m]). Z座標は0でなければならない.
    std::vector<Eigen::Vector2d> defaultStrideLimitationHull = std::vector<Eigen::Vector2d>{Eigen::Vector2d(0.15,-0.18),Eigen::Vector2d(-0.15,-0.18),Eigen::Vector2d(-0.15,-0.35),Eigen::Vector2d(0.15,-0.35)}; // 単位[m]. defaultのfootstepの、遊脚のエンドエフェクタの着地位置の範囲の凸包. 反対の脚のEndEffector frame(Z軸は鉛直)で表現した着地可能領域(自己干渉やIKの考慮が含まれる). Z成分は0でないといけない. 凸形状で,上から見て半時計回り. thetaの影響はlegHullとlegCollisionMarginを用いて別で評価されるので、defaultStrideLimitationHullでは考慮しなくて良い. 左右方向にsteppable regionをまたぐ場合は、これのY成分が大きくないと後ろ足がまたげない
    double defaultSwingVelocityRatio = 0.5; // 0~1. maxSwingVelocityの何倍か

    double maxSwingXYVelocity = 1.0; // 0より大きい. 単位[m/s].
    double maxSwingLandVelocity = 0.5; // 0より大きい[m/s]. touchVelと同じくらい?
    double maxSwingLiftVelocity = 0.5; // 0より大きい[m/s]. 0.2m登るときに一歩あたり1.4sくらい?
    double maxSwingThetaVelocity = 1.0; // 0より大きい. 単位[rad/s].
    double strideLimitationMaxTheta = 0.261799 + 0.01; // footstepの旋回上限. 支持脚相対. default 15[deg]. 0以上. 足裏同士の干渉は自動で回避できるが、膝同士の干渉はIK以外では回避できないので、内股方向には小さくすること.
    double strideLimitationMinTheta = -0.785398 - 0.01; // footstepの下限. 支持脚相対. default -45[deg]. 0以下. 足裏同士の干渉は自動で回避できるが、膝同士の干渉はIK以外では回避できないので、内股方向には小さくすること.
    std::vector<Eigen::Vector2d> strideLimitationHull = std::vector<Eigen::Vector2d>{Eigen::Vector2d(0.35,-0.15),Eigen::Vector2d(-0.35,-0.15),Eigen::Vector2d(-0.35,-0.350),Eigen::Vector2d(-0.20,-0.45),Eigen::Vector2d(0.20,-0.45),Eigen::Vector2d(0.35,-0.350)}; // footstepの上下限の凸包. 反対の脚のEndEffector frame(Z軸は鉛直)で表現した着地可能領域(自己干渉やIKの考慮が含まれる). Z成分は0でないといけない. 凸形状で,上から見て半時計回り. thetaの影響はhullとcollisionMarginを用いて別で評価されるので、strideLimitationHullでは考慮しなくて良い. 斜め方向の角を削るなどして、IKが解けるようにせよ.
    double collisionMargin = 0.02; // [m]. 左右の足のhullがこの距離以上離れるようにする. 0以上.
    double maxLandingHeight = 0.25; // [m]. 反対の脚のEndEffector frame(Z軸は鉛直)で表現した着地高さの上限(自己干渉やIKの考慮が含まれる).
    double minLandingHeight = -0.25; // [m]. 反対の脚のEndEffector frame(Z軸は鉛直)で表現した着地高さの下限(自己干渉やIKの考慮が含まれる).
    double goalOffset = -0.05; // [m]. 遊脚軌道生成時に、次に着地する場合、鉛直方向に, 目標着地位置に対して加えるオフセット. 0以下. 遅づきに対応するためのもの. 位置制御だと着地の衝撃が大きいので0がよいが、トルク制御時や、高低差がある地形や、衝撃を気にする必要がないシミュレーションでは-0.05等にした方がよい.
    double maxSwingTime = 2.0; // [s]. 0より大きい
    double resolutionTheta = 0.087266; // 5deg
    double resolutionTime = 0.05; // [s]
    double resolutionXY = 0.02; // [m]
    double liftXYThre1 = 0.015; // [m] resolution程度に大きくておく
    double liftXYThre2 = 0.05; // [m]
    double liftThetaThre = 0.087266; // [m] resolution程度に大きくしておく
    double liftRatioThre = 1.0; // 0より大きい
    double delayTimeOffset = 0.2; // [s]

    // 出力用
    double muTrans = 0.5; // 大股で歩くときはZMP-COMの位置関係的に、垂直抗力と並進力の比がそこまで大きな差にならないので、0.1~0.3程度だと足りない場合がある.
    double muRot = 0.05; // 旋回歩行時に必要なので、小さすぎてはいけない
    double regionMargin = 0.05; // legHullの周囲[m]をregionとする

    // 出力用 hullから自動計算
    Eigen::MatrixXd wrenchC; // endeffector frame. link1がlink2から受ける力に関する接触力制約. ?x6
    cnoid::VectorX wrenchld;
    cnoid::VectorX wrenchud;
    Region3D region; // endeffector frame.
    double stepHeight = 0.05; // footstepの足上げ高さ[m]. 0以上

    void updateFromHull();
    void flipY(EEParam& param); //このEEParamのY成分を反転させてparamにコピーする. nameとparentLink以外
  };

  class NominalEE {
  public:
    std::string name;
    cnoid::LinkPtr link;
    cnoid::Isometry3 localPose = cnoid::Isometry3::Identity();

    cnoid::LinkPtr frameLink; // ""ならworldではなくfootOrigin
    cnoid::Isometry3 framePose = cnoid::Isometry3::Identity();
    double time = 1.0;
    cnoid::Isometry3 pose = cnoid::Isometry3::Identity();
    std::vector<bool> freeAxis = std::vector<bool>(6,false);
    int priority = 1; // 0 or 1. 0ならふつう. 1は重心と同じ

    bool updateFromIdl(const State& state, const actkin_balancer::ActKinBalancerService::NominalEEIdl& idl);
    void convertToIdl(const State& state, actkin_balancer::ActKinBalancerService::NominalEEIdl& idl);
  };

  class NominalInfo {
  public:
    double nominalqTime = 1.0;
    cnoid::VectorX nominalq;
    std::vector<NominalEE> nominalEE;
    double nominalZ = 1.0;

    bool updateFromIdl(const State& state, const actkin_balancer::ActKinBalancerService::NominalIdl& idl);
    void convertToIdl(const State& state, actkin_balancer::ActKinBalancerService::NominalIdl& idl);
  };


  // このクラスのメンバ変数は、全てfiniteである(nanやinfが無い)ことが仮定されている. 外部から値をセットするときには、finiteでない値を入れないようにすること
  class State {
  public:
    // from data port. 狭義のstate
    cnoid::BodyPtr robot; // actual.
    cnoid::Vector3 cogVel = cnoid::Vector3::Zero();
    std::vector<std::shared_ptr<Contact> > contacts; // actual

    // objects

    // environments
    std::vector<std::vector<cnoid::Vector3> > steppableRegion; // world frame. 着地可能領域の凸包の集合. 要素数0なら、全ての領域が着地可能として扱われる.
    std::vector<double> steppableHeight; //  world frame. 要素数と順序はsteppableRegionと同じ。steppableRegionの各要素の重心Z.

    // legContacts
    std::vector<bool> actContact = std::vector<bool>{false,false};// rleg, lleg. contactsから計算されるactual contact
  public:
    std::unordered_map<std::string, cnoid::LinkPtr> nameLinkMap; // MODE_ABC中はconstant. URDFのLink名 -> linkPtr
    std::unordered_map<cnoid::LinkPtr, std::string> linkNameMap; // MODE_ABC中はconstant. linkPtr -> URDFのLink名
    const double g = 9.80665; // constant. 重力加速度
  public:
    // parameters. contactable pointの情報.
    std::vector<EEParam> ee = std::vector<EEParam>(NUM_LEGS); // rleg, lleg

    NominalInfo nominal;
  public:

    // RTC起動時に一回呼ばれる.
    void init(const cnoid::BodyPtr& robot_);

    // startBalancer時に呼ばれる
    bool onStartBalancer();

    // MODE_ST中のみ呼ばれる
    void updateRobotFromIdl(const RTC::TimedDoubleSeq& m_qAct, const RTC::TimedDoubleSeq& m_dqAct, const RTC::TimedPose3D& m_actBasePose, const RTC::TimedVelocity3D& m_actBaseVel, double dt);
    void updateContactFromIdl(const contact_state_msgs::TimedContactSeq& m_actContactState);
  };

};

#endif
