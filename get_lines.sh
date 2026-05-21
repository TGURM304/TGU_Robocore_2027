git ls-files . ':!build/' ':!tools/foxglove/' ':!tools/tomlpp.hpp' ':!tools/concurrentqueue.hpp' ':!tools/BS_thread_pool.hpp'  ':!io/hikrobot/hikSDK'  ':!*.md' ':!*.txt' ':!*.toml' | xargs wc -l
