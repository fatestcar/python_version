#include <iostream>
using namespace std;
void prin (int a,int b)
{
    cout << a << " " << b << endl;
}
extern "C"
{
   void cPrin (int a,int b)
 {
        prin(a,b);
 }
}