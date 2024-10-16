#pragma once

namespace dymp{
namespace mpc{

class ThreadImpl;

/** wrapper of std::thread
 **/
class Thread{
protected:
	ThreadImpl* impl;

public:
	void Run ();
	bool Join();

	void operator()();
	
	virtual void Func() = 0;

	static void SleepUS(int us);
	
	Thread();
	virtual ~Thread();
};

}
}
