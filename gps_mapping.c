#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main() {
    FILE *file = fopen("C:/Users/gram/Desktop/Univercity/Grade3/2/Capstone design/gps_data.txt", "r");
    if (file == NULL) {
        printf("������ �� �� �����ϴ�.\n");
        return 1;
    }
    FILE* outputFile = fopen("C:/Users/gram/Desktop/Univercity/Grade3/2/Capstone design/output.txt", "w");
    if (outputFile == NULL) {
        printf("��� ������ �� �� �����ϴ�.\n");
        fclose(file);
        return 1;
    }

    char line[256];
    double latitude = 0.0, longitude = 0.0;

    while (fgets(line, sizeof(line), file)) {
        // "Latitude:"�� "Longitude:"�� ��� ���Ե� �ٿ��� ������ �浵 �� ����
        if (strstr(line, "Latitude:") && strstr(line, "Longitude:")) {
            sscanf(line, "Latitude: %lf Longitude: %lf", &latitude, &longitude);
            printf("Latitude: %.6f, Longitude: %.6f\n", latitude, longitude);

            // ������ ������ �浵�� output.txt ���Ͽ� ����
            fprintf(outputFile, "Latitude: %.6f, Longitude: %.6f\n", latitude, longitude);
        }
    }

    fclose(file);
    fclose(outputFile);

    printf("����� ������ �浵 ���� output.txt ���Ͽ� ����Ǿ����ϴ�.");
    return 0;
}
