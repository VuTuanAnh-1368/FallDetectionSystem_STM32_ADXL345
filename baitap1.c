/* 1  Cho 1 mảng 10 Int phần tử , 
tìm 3 số bất kỳ trong mảng mà tổng bằng 0 */

#include <stdio.h>

void Find_5_Number(int arr[], int size) {
    int brr[3];
    for(int i = 0; i < size - 2; i++) {
        brr[0] = arr[i];
        for(int j = i + 1; j < size -1; j++) {
            brr[1] = arr[j];
            for(int k = j + 1; k < size; k++) {
                if(brr[0] + brr[1] + arr[k] == 0) {
                    brr[2] = arr[k];
                    printf("\n(%d, %d, %d) ",brr[0], brr[1], brr[2]); 
                }
            }
        }
    } 
}

int main() {
    int Num_serie[10];
    for(int i = 0; i < 10; i++) {
        printf("Nhap phan tu thu %d: ", i + 1);
        scanf("%d", &Num_serie[i]);
    }
    Find_3_Number(Num_serie, 10);
    return 0;
}