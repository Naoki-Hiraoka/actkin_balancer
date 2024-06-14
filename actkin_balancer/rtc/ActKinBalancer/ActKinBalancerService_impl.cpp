#include "ActKinBalancerService_impl.h"
#include "ActKinBalancer.h"

ActKinBalancerService_impl::ActKinBalancerService_impl()
{
}

ActKinBalancerService_impl::~ActKinBalancerService_impl()
{
}

CORBA::Boolean ActKinBalancerService_impl::startBalancer()
{
  return this->comp_->startBalancer();
}

CORBA::Boolean ActKinBalancerService_impl::stopBalancer()
{
  return this->comp_->stopBalancer();
}

CORBA::Boolean ActKinBalancerService_impl::setActKinBalancerParam(const actkin_balancer::ActKinBalancerService::ActKinBalancerParam& i_param)
{
  return this->comp_->setActKinBalancerParam(i_param);
};

CORBA::Boolean ActKinBalancerService_impl::getActKinBalancerParam(actkin_balancer::ActKinBalancerService::ActKinBalancerParam_out i_param)
{
  i_param = new actkin_balancer::ActKinBalancerService::ActKinBalancerParam();
  return this->comp_->getActKinBalancerParam(*i_param);
};


CORBA::Boolean ActKinBalancerService_impl::setRefState(const actkin_balancer_msgs::RefStateIdl& i_param)
{
  return this->comp_->setRefState(i_param);
}

void ActKinBalancerService_impl::setComp(ActKinBalancer *i_comp)
{
  comp_ = i_comp;
}
