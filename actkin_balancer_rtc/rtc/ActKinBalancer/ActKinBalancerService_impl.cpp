#include "ActKinBalancerService_impl.h"
#include "ActKinBalancer.h"

ActKinBalancerService_impl::ActKinBalancerService_impl()
{
}

ActKinBalancerService_impl::~ActKinBalancerService_impl()
{
}

void ActKinBalancerService_impl::setComp(ActKinBalancer *i_comp)
{
  comp_ = i_comp;
}

CORBA::Boolean ActKinBalancerService_impl::templateParam(const CORBA::Double data)
{
  return comp_->templateParam(data);
};
