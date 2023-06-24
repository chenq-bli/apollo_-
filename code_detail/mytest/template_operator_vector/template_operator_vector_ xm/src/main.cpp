#include "memory"
#include "template_vec.h"

using namespace MySTL;

int main()
{
    // Vector<string> vecs;
    // vecs.push_back("Chen");
    // vecs.push_back("Micheal");
    // vecs.push_back("!");

    // cout << vecs[0] << " " << vecs[1] << " " << vecs[2] << endl;

    // Vector<int> v2(3, 2);
    // cout << v2[0] << " " << v2[1] << " " << v2[2] << endl;

    // Vector<string> v3(vecs);
    // cout << v3[0] << " " << v3[1] << " " << v3[2] << endl;
    // v3.pop_back();
    // v3.pop_back();
    // Vector<int> v4 = v2;
    // cout << v4[0] << " " << v4[1] << " " << v4[2] << endl;

    // Vector<string> v5(3, "you");
    // cout << v5[0] << " " << v5[1] << " " << v5[2] << endl;
    
    std::unique_ptr<Vector<string>>vec_ptr1(new Vector<string>(3, "me"));

    std::shared_ptr<Vector<string>>vec_ptr2;
    vec_ptr2=std::shared_ptr<Vector<string>>(new Vector<string>(3, "ha"));

    std::shared_ptr<Vector<string>>vec_ptr3=std::make_shared<Vector<string>>(3, "love");


    vec_ptr1->push_back("好人啊!");
    vec_ptr2->pop_back();
    vec_ptr2->push_back("!");
    vec_ptr2->push_back("!");
    vec_ptr3->push_back("you");
    cout << (*vec_ptr1)[0] << " " << (*vec_ptr1)[1] << " " << (*vec_ptr1)[2] << " " << (*vec_ptr1)[3]<< endl;
    cout << vec_ptr2->at(0) << " " << vec_ptr2->at(1) << " " << vec_ptr2->at(2) << " " << vec_ptr2->at(3)<< endl;
    cout << vec_ptr3->at(0) << " " << vec_ptr3->at(1) << " " << vec_ptr3->at(2) << " " << vec_ptr3->at(3)<< endl;


    Vector<string> * p1= vec_ptr1.release();
    cout << p1->at(0) << " " << p1->at(1) << " " << p1->at(2) << " " << p1->at(3)<< endl;
    vec_ptr1.reset(new Vector<string>(3, "tian"));
    vec_ptr1->push_back("好人啊!");
    cout << vec_ptr1->at(0) << " " << vec_ptr1->at(1) << " " << vec_ptr1->at(2) << " " << vec_ptr1->at(3)<< endl;
    
    std::unique_ptr<Vector<string>>vec_ptr4=std::move(vec_ptr1);
    cout << vec_ptr4->at(0) << " " << vec_ptr4->at(1) << " " << vec_ptr4->at(2) << " " << vec_ptr4->at(3)<< endl;
    // cout << vec_ptr1->at(0) << " " << vec_ptr1->at(1) << " " << vec_ptr1->at(2) << " " << vec_ptr1->at(3)<< endl;
    vec_ptr1.reset(new Vector<string>(3, "yo"));
    vec_ptr1->push_back("好人啊!");
    vec_ptr1.swap(vec_ptr4);
    cout << vec_ptr4->at(0) << " " << vec_ptr4->at(1) << " " << vec_ptr4->at(2) << " " << vec_ptr4->at(3)<< endl;


    std::shared_ptr<Vector<string>>vec_ptr5=vec_ptr2;
    cout << vec_ptr5->at(0) << " " << vec_ptr5->at(1) << " " << vec_ptr5->at(2) << " " << vec_ptr5->at(3)<< endl;
    cout<<vec_ptr5.use_count()<<endl;
    vec_ptr5=std::shared_ptr<Vector<string>>(new Vector<string>(3, "wuren"));
    cout<<vec_ptr2.use_count()<<endl;
    std::shared_ptr<Vector<string>>vec_ptr6=std::make_shared<Vector<string>>(3, "love");
    vec_ptr6->push_back("you");
    std::weak_ptr<Vector<string>>w_ptr(vec_ptr6);
    std::shared_ptr<Vector<string>>vec_ptr7=w_ptr.lock();
    if(vec_ptr7)
    {
        cout << "weak_ptr7:" << vec_ptr7->at(0) << " " << vec_ptr7->at(1) << " " << vec_ptr7->at(2) << " " << vec_ptr7->at(3)<< endl;
    
    }
    else
    {
        cout<<"错误发生！对象已经被释放！"<<endl;
    }
    vec_ptr6.reset();
    vec_ptr7.reset();
    std::shared_ptr<Vector<string>>vec_ptr8=w_ptr.lock();
    if(vec_ptr8)
    {
        cout << "weak_ptr8:" << vec_ptr7->at(0) << " " << vec_ptr7->at(1) << " " << vec_ptr7->at(2) << " " << vec_ptr7->at(3)<< endl;
    
    }
    else
    {
        cout<<"错误发生！对象已经被释放！"<<endl;
    }
    









}