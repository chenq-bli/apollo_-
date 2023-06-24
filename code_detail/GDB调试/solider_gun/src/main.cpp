#include "solider_and_gun.h"

int main()
{
    // int a=0;
    Solider chenqiang("陈强");
    Gun *AK47 = new Gun("AK47");
    chenqiang.take_gun(AK47);
    chenqiang.add_bullet_togun(2);
    for (int i = 0; i < 4; i++)
    {
        chenqiang.fire();
    }
    // AK47->printfinfo();
    return 0;

}