## C++ Graph Library

This library consists of *.h and *.cpp files, most-importantly: Graph.h and Graph.cpp. Including these files will allow the user to instantiate 'Graph' objects to model graph-related problems. Graphs are persistent (ie they can be written to disk as *.txt files for later retrieval through a read-from-file Graph constructor). 

No external libraries (eg Boost) were used in this project, except for the C++ Standard Template Library.

## Use Example
```c++
#include "Graph.h"
using namespace std;

int main()
{
    //Declare a Graph object, populate it.	
    Graph G;
    G.addNode("A"); 
    G.addNode("B");
    G.addEdge("A","B");
    ...
    vector<string> shortestPath = G.BFS("A","Z");
    Graph MST = G.Prims(); //Creates new Graph object from G
    MST.saveGraph("MinimumSpanningTree"); //Write MST to disk

    //Retrieve Graph object from .txt file
    Graph F(string("NetworkConnections.txt"));
}

$ g++ --std=c++11 main.cpp Node.cpp Graph.cpp -o main
```



## Installation
Simply download the .h and .cpp files, place them in your working directory, and "#include" them in your appropiate files.


## MIT License

Copyright (c) 2021 minyc510

```
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
