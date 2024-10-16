#pragma once

#include "criticalsection.h"

#include <string>
#include <vector>
#include <map>

namespace dymp{
namespace mpc{

class EventGroup;

/**
	event object
 */
class Event{
public:
	std::string               name;
	CriticalSection           cs;
	std::vector<EventGroup*>  groups;

public:
	bool	Create(bool manual = false);

	void	Close();

	/** wait event
	 */
	bool Wait(int timeout = 1000);

	/** set event
	 */
	void Set();

	/** check if set
	 */
	bool IsSet();

	/** reset event
	 */
	void Reset();

	Event();
	~Event();
};

/**
	group of event objects
 */
class EventGroup : public Event, public std::vector<Event*>{
public:
	void Add   (Event* ev);
	void Remove(Event* ev);

	int Wait(int timeout);

	 EventGroup();
	~EventGroup();
};

}
}
