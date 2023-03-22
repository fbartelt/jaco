# Prerequisites

1. [dqrobotics development branch](https://dqrobotics.github.io/)

  - C++:
    ```shell
    sudo add-apt-repository ppa:dqrobotics-dev/development
    sudo apt-get update
    sudo apt-get install libdqrobotics
    ```
  - Python:  (Known issue in Ubuntu [here](https://github.com/dqrobotics/python/issues/44))
    ```shell
    python3 -m pip install --user --pre dqrobotics  
    ```  

2. Interface with CoppeliaSim (Formely V-REP). Check the [documentation](https://dqroboticsgithubio.readthedocs.io/en/latest/index.html) 
3. [CoppeliaSim](https://www.coppeliarobotics.com/) 
