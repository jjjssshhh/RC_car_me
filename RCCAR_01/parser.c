#include <stdio.h>
#include <stdlib.h>

int main() {
    // 실제 파일명인 fast_00.txt로 열기
    FILE *rfp = fopen("normal_02.txt", "r");
    FILE *wfp = fopen("autodata.h", "w");

    if (rfp == NULL) {
        perror("fast_00.txt 파일을 찾을 수 없습니다");
        return 1;
    }
    if (wfp == NULL) {
        perror("헤더 파일 생성 실패");
        fclose(rfp);
        return 1;
    }

    char line[256];
    fprintf(wfp, "#ifndef AUTODATA_H\n#define AUTODATA_H\n\n");
    fprintf(wfp, "#include <stdint.h>\n\n");

    fprintf(wfp, "typedef struct {\n");
    fprintf(wfp, "    int32_t flag;\n");
    fprintf(wfp, "    int32_t sonic_L, sonic_M, sonic_R;\n");
    fprintf(wfp, "    int32_t v_l, v_r;\n");
    fprintf(wfp, "    int32_t interval;\n");
    fprintf(wfp, "} DriveStep;\n\n");

    fprintf(wfp, "const DriveStep track_data[] = {\n");

    int count = 0;
    while (fgets(line, sizeof(line), rfp)) {
        int flag, s1, s2, s3, v1, v2, inter;

        /* sscanf 수정 포인트:
           %*s : 첫 번째 단어(10:21:38.161 같은 시간 정보)를 읽고 버립니다.
           %d,%d,%d,%d,%d,%d,%d : 그 뒤의 숫자 7개를 읽습니다.
        */
        if (sscanf(line, "%*s %d,%d,%d,%d,%d,%d,%d", &flag, &s1, &s2, &s3, &v1, &v2, &inter) == 7) {
            fprintf(wfp, "    {%d, %d, %d, %d, %d, %d, %d},\n", flag, s1, s2, s3, v1, v2, inter);
            count++;
        }
    }

    fprintf(wfp, "};\n\n");
    fprintf(wfp, "#define TRACK_DATA_SIZE %d\n\n", count);
    fprintf(wfp, "#endif\n");

    fclose(rfp);
    fclose(wfp);
    
    if (count > 0) {
        printf("성공! 총 %d개의 데이터를 'autodata.h'로 변환했습니다.\n", count);
    } else {
        printf("데이터를 찾지 못했습니다. 파일 내용의 형식을 확인하세요.\n");
    }

    return 0;
}
