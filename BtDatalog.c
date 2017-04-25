/* 2013/08/17 created by k.nakagawa bluetooth�Ńf�[�^���O�o�͂��� */
#include "BtDatalog.h"

#include "ecrobot_interface.h"

static U8 data_log_buffer[DATA_LOG_BUFFER_LEN];
static U16 count = 0;

void bt_data_log_init()
{
	count = 0;
}

//�f�[�^���O���o�b�t�@�ɒǉ�(1���4�o�C�g�����ǉ������)
//0�o�C�g��		�FDataID
//1�o�C�g��		�Ftime
//2-3�o�C�g��	�F���Z���T�l
//4�o�C�g��		�F���[�^�G���R�[�_�l�iB�j
//5�o�C�g��		�F���[�^�G���R�[�_�l�iC�j
S8 bt_data_log_append(U8 dataID, S8 Data1, S8 Data2, U16 lightIn)
{
	switch(dataID)
	{
		case 1:
			//�i�o�b�t�@�T�C�Y��6�j�񕪃f�[�^������
			if(count < (U16)(DATA_LOG_BUFFER_LEN/6))
			{
				*(( U8 *)(&data_log_buffer[count*6+0]))  = (U8)dataID;
				*(( U8 *)(&data_log_buffer[count*6+1]))  = (U8)(systick_get_ms() / 100);		//U32��U8
				*((S16 *)(&data_log_buffer[count*6+2]))  = (S16)sensor_adc(2);
				*(( U8 *)(&data_log_buffer[count*6+4]))  = (U8)nxt_motor_get_count(1); 	//S32��U8
				*(( U8 *)(&data_log_buffer[count*6+5]))  = (U8)nxt_motor_get_count(2); 	//S32��U8

				count++;

				return 0;
			}
			else
			{
				//�G���[
				return -1;
			}
			break;
		case 2:
			//�i�o�b�t�@�T�C�Y��6�j�񕪃f�[�^������
			if(count < (U16)(DATA_LOG_BUFFER_LEN/6))
			{
				*(( U8 *)(&data_log_buffer[count*6+0]))  = (U8)dataID;
				*(( U8 *)(&data_log_buffer[count*6+1]))  = (U8)(systick_get_ms() / 100);		//U32��U8
				*(( S8 *)(&data_log_buffer[count*6+2]))  = (S8)Data1;
				*(( S8 *)(&data_log_buffer[count*6+3]))  = (S8)Data2;
				*(( U8 *)(&data_log_buffer[count*6+4]))  = (S16)sensor_adc(2);

				count++;

				return 0;
			}
			else
			{
				//�G���[
				return -1;
			}
			break;
		case 3:
			//�i�o�b�t�@�T�C�Y��6�j�񕪃f�[�^������
			if(count < (U16)(DATA_LOG_BUFFER_LEN/6))
			{
				*(( U8 *)(&data_log_buffer[count*6+0]))  = (U8)dataID;
				*(( U8 *)(&data_log_buffer[count*6+1]))  = (U8)(systick_get_ms() / 100);		//U32��U8
				*((S16 *)(&data_log_buffer[count*6+2]))  = (S16)sensor_adc(2);
				*((U16 *)(&data_log_buffer[count*6+4]))  = (U16)lightIn;

				count++;

				return 0;
			}
			else
			{
				//�G���[
				return -1;
			}
			break;
		default:
			return -1;
			break;
	}

	return -1;
}

//�f�[�^���O��PC�֑��M�i4ms�������x�ŌĂяo���j
S8 bt_data_log_send()
{
	//�o�b�t�@�ɓ����Ă���f�[�^�����[�v����
	static U16 i = 0;
	ecrobot_send_bt(data_log_buffer, i*6, 6);
	i++;
	if(i < count)
	{
		return 0;
	}
	else
	{
		//�G���[
		return -1;
	}
}
