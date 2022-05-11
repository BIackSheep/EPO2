#include <stdio.h>

int main() {
    int p=0,q=0;
    int test[4] = {1,2,3,4};
    int testint=0;
    int nr_of_stations = 4;
    while(1) {
     for(p=nr_of_stations-1;p>=1;p--) {
        for(q=p-1;q>=0;q--) {
            if(test[p]>test[q]) {
            testint = test[q];
            test[q] = test[p];
            test[p] = testint;
            break;
            }
        }
        if(test[p]<test[q]) {
            break;
        }
     }
     printf("p=%i, q=%i,  {%i,%i,%i,%i}\n",p,q,test[0],test[1],test[2],test[3]);
     getchar();
    }
 return 0;
}
