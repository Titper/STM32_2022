#include "caracter.h"

void Reverse(char *str, int Length)
{
    int i = 0;
    int j = Length-1;
    int Temp;

    while (i<j)
    {
        Temp = str[i];
        str[i] = str[j];
        str[j] = Temp;
        i++; j--;
    }
}

int IntToStr(int x, char str[], int d)//Conversion entier vers string
{
    int i = 0;
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }

    while (i < d)
        str[i++] = '0';

    Reverse(str, i);
    str[i] = '\0';
    return i;
}

void FloatToStr(float n, char *res, int Decimal)//Conversion réel vers string
{
    int ipart = (int)n;
    float fpart = n - (float)ipart;
    int i = IntToStr(ipart, res, 0);

    if (Decimal != 0)
    {
        res[i] = '.';
        fpart = fpart * pow(10, Decimal);
        IntToStr((int)fpart, res + i + 1, Decimal);
    }
}

