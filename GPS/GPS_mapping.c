#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main() {
    FILE *file = fopen("C:/Users/gram/Desktop/Univercity/Grade3/2/Capstone design/gps_data.txt", "r");
    if (file == NULL) {
        printf("파일을 열 수 없습니다.\n");
        return 1;
    }
    FILE* outputFile = fopen("C:/Users/gram/Desktop/Univercity/Grade3/2/Capstone design/output.txt", "w");
    if (outputFile == NULL) {
        printf("출력 파일을 열 수 없습니다.\n");
        fclose(file);
        return 1;
    }

    char line[256];
    double latitude = 0.0, longitude = 0.0;

    while (fgets(line, sizeof(line), file)) {
        // "Latitude:"와 "Longitude:"가 모두 포함된 줄에서 위도와 경도 값 추출
        if (strstr(line, "Latitude:") && strstr(line, "Longitude:")) {
            sscanf(line, "Latitude: %lf Longitude: %lf", &latitude, &longitude);
            printf("Latitude: %.6f, Longitude: %.6f\n", latitude, longitude);

            // 추출한 위도와 경도를 output.txt 파일에 저장
            fprintf(outputFile, "Latitude: %.6f, Longitude: %.6f\n", latitude, longitude);
        }
    }

    fclose(file);
    fclose(outputFile);

    printf("추출된 위도와 경도 값이 output.txt 파일에 저장되었습니다.");
    return 0;
}
