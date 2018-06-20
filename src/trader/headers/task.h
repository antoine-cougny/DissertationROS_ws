#pragma once

#ifndef __TASK_H__
#define __TASK_H__

#include <string>

class Task
{
  public:
    void printInfo();

  private:
    std::string m_id_task;
    std::string m_type_task;
    int m_priority;
    int m_number_of_robots;
    std::string m_resources_needed;
    std::string m_other_info;
    int value_reward;
    bool m_is_done;

};
#endif
