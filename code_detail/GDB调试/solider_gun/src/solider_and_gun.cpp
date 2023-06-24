#include "solider_and_gun.h"

void Gun::add_bullet(int c)
{
    this->_bullet_count=_bullet_count+c;
}
bool Gun::shoot()
{
    if(this->_bullet_count<=0)
    {
        std::cout<<"没有子弹了！"<<std::endl;
        return false;
    }
    else
    {
        this->_bullet_count=_bullet_count-1;
        std::cout<<"发射成功，还剩"<<this->_bullet_count<<"枚子弹"<<std::endl;
        return true;
    }
    
}
void Gun::printfinfo()
{
        std::cout<<"Now剩余子弹数量："<<this->_bullet_count<<"颗"<<std::endl;
}

void Solider::take_gun(Gun* thegun)
{
    this->_ptr_gun=thegun;
}



void Solider::add_bullet_togun(int zd)
{
    this->_ptr_gun->add_bullet(zd);
}
void Solider::fire()
{
    this->_ptr_gun->shoot();
}
Solider::~Solider()
{
    if(this->_ptr_gun==nullptr)
    {
        return;
    }
    else{
        delete this->_ptr_gun;
        this->_ptr_gun=nullptr;
    }
}