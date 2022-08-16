#include "ParseJson.h"

char one[5],two[5],three[5],four[5],five[5],six[5];
char one_k210[5],two_k210[5],three_k210[5];

void json_Parse(char* str)
{
    cJSON *json,*json_one,*json_two,*json_three,*json_four,*json_five,*json_six;
//    char* str="{\"1\":\"11\",\"2\":\"22\",\"3\":\"33\",\"4\":\"44\",\"5\":\"55\",\"6\":\"66\"}";
 
    json = cJSON_Parse(str); //���õ����ַ���������json��ʽ
    /****************************/
    /*	  ���Խ�JSON��ӡ����	*/
    /***************************/
//		 char *out_data = cJSON_Print(json);   //��json��ʽ��ӡ�������ַ�����ʽ
//		 printf("%s",out_data);

    json_one = cJSON_GetObjectItem( json , "1" );  //��json��ȡ��ֵ����
    json_two = cJSON_GetObjectItem( json , "2" );//��json��ȡ��ֵ����
    json_three = cJSON_GetObjectItem( json , "3" );//��json��ȡ��ֵ����
    json_four = cJSON_GetObjectItem( json , "4" );//��json��ȡ��ֵ����
    json_five = cJSON_GetObjectItem( json , "5" );//��json��ȡ��ֵ����
    json_six = cJSON_GetObjectItem( json , "6" );//��json��ȡ��ֵ����

		printf("1:%s   2:%s   3:%s\r\n",json_one->valuestring,json_two->valuestring,json_three->valuestring);
		printf("4:%s   5:%s   6:%s\r\n",json_four->valuestring,json_five->valuestring,json_six->valuestring);
	
		strcpy(one,json_one->valuestring);
		strcpy(two,json_two->valuestring);
		strcpy(three,json_three->valuestring);
		strcpy(four,json_four->valuestring);
		strcpy(five,json_five->valuestring);
		strcpy(six,json_six->valuestring);

		printf("%s\r\n",one);
			
    cJSON_Delete(json);  //�ͷ��ڴ� 
}

void json_Parse_K210(char* str)
{
    cJSON *json,*json_one,*json_two,*json_three;
//    char* str="{\"1\":\"11\",\"2\":\"22\",\"3\":\"33\"}";
 
    json = cJSON_Parse(str); //���õ����ַ���������json��ʽ
    /****************************/
    /*	  ���Խ�JSON��ӡ����	*/
    /***************************/
//		 char *out_data = cJSON_Print(json);   //��json��ʽ��ӡ�������ַ�����ʽ
//		 printf("%s",out_data);
		printf("%s",str);
    json_one = cJSON_GetObjectItem( json , "1" );  //��json��ȡ��ֵ����
    json_two = cJSON_GetObjectItem( json , "2" );//��json��ȡ��ֵ����
    json_three = cJSON_GetObjectItem( json , "3" );//��json��ȡ��ֵ����

//		printf("1:%s   2:%s   3:%s\r\n",json_one->valuestring,json_two->valuestring,json_three->valuestring);
	
		strcpy(one_k210,json_one->valuestring);
		strcpy(two_k210,json_two->valuestring);
		strcpy(three_k210,json_three->valuestring);

    cJSON_Delete(json);  //�ͷ��ڴ� 
}

