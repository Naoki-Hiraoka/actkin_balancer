#ifndef ActKinBalancerSERVICESVC_IMPL_H
#define ActKinBalancerSERVICESVC_IMPL_H

#include "actkin_balancer/idl/ActKinBalancerService.hh"

class ActKinBalancer;

class ActKinBalancerService_impl
  : public virtual POA_actkin_balancer::ActKinBalancerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  ActKinBalancerService_impl();
  ~ActKinBalancerService_impl();

  CORBA::Boolean startBalancer();
  CORBA::Boolean stopBalancer();

  CORBA::Boolean setActKinBalancerParam(const actkin_balancer::ActKinBalancerService::ActKinBalancerParam& i_param);
  CORBA::Boolean getActKinBalancerParam(actkin_balancer::ActKinBalancerService::ActKinBalancerParam_out i_param);

  CORBA::Boolean setRefState(const actkin_balancer_msgs::RefStateIdl& i_param);

  void setComp(ActKinBalancer *i_comp);
private:
  ActKinBalancer *comp_;
};

#endif
