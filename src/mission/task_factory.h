#pragma once

#include <factory/static_registrator.h>
#include <config_reader/yaml_reader.h>
#include <libipc/ipc.h>
#include "task.h"

#include <functional>

using TaskInit = std::function<TaskBase* (const YamlReader&, ipc::Communicator&)>;

using RegisteredTasks = Singleton<LazyFactory<TaskBase, TaskInit> >;
using TaskRegistrator = StaticRegistrator<LazyFactory<TaskBase, TaskInit>, TaskInit>;

#define REGISTER_TASK(CLASSNAME, CONFIG_NAME)       \
TaskRegistrator CLASSNAME##CONFIG_NAME(#CONFIG_NAME, \
    [](const YamlReader& cfg, ipc::Communicator& comm) { return new CLASSNAME(cfg, comm); });

