#include"template_vec.h"

using namespace MySTL;


int main()
{
    Vector<string> vecs;
    vecs.push_back("Chen");
    vecs.push_back("Micheal");
    vecs.push_back("!");

    cout<<vecs[0]<<" "<<vecs[1]<<" "<<vecs[2]<<endl;

    Vector<int> v2(3,2);
    cout<<v2[0]<<" "<<v2[1]<<" "<<v2[2]<<endl;

    Vector<string> v3(vecs);
    cout<<v3[0]<<" "<<v3[1]<<" "<<v3[2]<<endl;
    v3.pop_back();
    v3.pop_back();
    Vector<int> v4= v2;
    cout<<v4[0]<<" "<<v4[1]<<" "<<v4[2]<<endl;

    Vector<string> v5(3,"you");
    cout<<v5[0]<<" "<<v5[1]<<" "<<v5[2]<<endl;

}