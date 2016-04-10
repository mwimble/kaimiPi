#include "StrategyContext.h"

StrategyContext& StrategyContext::Singleton() {
	static StrategyContext singleton_;
	return singleton_;
}
