#ifndef ActKinBalancerSERVICESVC_IMPL_H
#define ActKinBalancerSERVICESVC_IMPL_H

#include "actkin_balancer_rtc/idl/ActKinBalancerService.hh"

class ActKinBalancer;

class ActKinBalancerService_impl
  : public virtual POA_OpenHRP::ActKinBalancerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  ActKinBalancerService_impl();
  ~ActKinBalancerService_impl();

  CORBA::Boolean templateParam(const CORBA::Double data);
  void setComp(ActKinBalancer *i_comp);
private:
  ActKinBalancer *comp_;
};

#endif
