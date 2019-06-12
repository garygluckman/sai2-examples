# sai2-examples

sai2-examples is a collection of examples that demonstrate different aspects of the SAI2 simulation framework. Examples include:
* Non-redundant arm (Puma)
* Redundant arm (Kuka)

sai2-examples depends on [sai2-interfaces](https://github.com/manips-sai-org/sai2-interfaces). 
Clone it and ensure that its definitions are exported.

To build the examples:
```
mkdir build && cd build
cmake ..
make
```

Binaries will placed in the `bin/` folder. Inside, you should see something like:
```
sai2-examples/bin/01-non_redundant_arm$ ls
01-non_redundant_arm.html  controller01  interface  resources  simviz01
```

To fully start this example, run the following commands in separate terminal windows:
```
python interface/server.py start 01-non_redundant_arm.html
./simviz01
./controller01
```

After that's all done, navigate to `localhost:8000` to interact with the example.

