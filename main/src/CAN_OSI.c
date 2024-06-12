#include <CAN_OSI.h>
#include <string.h>
#include "CRC.h"
#include "CAN_Flag.h"
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/twai.h"
void CAN_ProcessRxBuffer(FlagFrameHandle *FlagHandle, uint8_t ID,
		CANBufferHandleStruct *RxBuffer, uint8_t *DataPhysical,
		FlagRecNotification *FlagRecHandle) {
	uint8_t FrameType = 0;
	RxBuffer->NodeHandle[ID].NodeIndex++;
	for (; FrameType < RxBuffer->NodeHandle[ID].NumberOfFrame; FrameType++) {
		CAN_ProcessFrame(FlagHandle, ID, RxBuffer, FrameType, DataPhysical);
	}
	if (RxBuffer->NodeHandle[ID].NodeIndex
			== RxBuffer->NodeHandle[ID].NumberOfFrame) {
		if (FlagHandle->FlagID[ID].SumOfFlag
				== RxBuffer->NodeHandle[ID].NumberOfFrame) {
			*FlagRecHandle = REC_FRAMEDATA_SUCCESS;
			RxBuffer->NodeHandle[ID].NodeIndex = 0;
			RxBuffer->NodeHandle[ID].DuplicateFrame = 0;
			FlagHandle->FlagID[ID].SumOfFlag = 0;
			for (FrameType = 0;
					FrameType < RxBuffer->NodeHandle[ID].NumberOfFrame;
					FrameType++) {
				FlagHandle->FlagID[ID].FlagFrameFull[FrameType] = 0;
			}
		} else {
			*FlagRecHandle = REC_FRAMEDATA_ERROR;
			for (FrameType = 0;
					FrameType <= RxBuffer->NodeHandle[ID].NumberOfFrame;
					FrameType++) {
				if (FlagHandle->FlagID[ID].FlagFrameFull[FrameType] == 0) {
					FlagHandle->FlagID[ID].FrameError[FrameType] = 1;
					FlagHandle->ID = ID;
					FlagHandle->NumberOfFrame =
							RxBuffer->NodeHandle[ID].NumberOfFrame;
					// CAN_Receive_Error_Handle(FlagRecHandle, FlagHandle);
				}
			}
		}
	}
}

void CAN_ProcessFrame(FlagFrameHandle *FlagHandle, uint8_t ID,
		CANBufferHandleStruct *RxBuffer, uint8_t FrameType, uint8_t *Data) {
	if (RxBuffer->NodeHandle[ID].FrameType == FrameType
			&& FlagHandle->FlagID[ID].FlagFrameFull[FrameType] == 0) {
		memcpy(
				RxBuffer->NodeHandle[ID].NodeBuffer[RxBuffer->NodeHandle[ID].FrameType],
				Data, CAN_MAX_DATA);

		FlagHandle->FlagID[ID].FlagFrameFull[FrameType] = 1;
		FlagHandle->FlagID[ID].SumOfFlag +=
				FlagHandle->FlagID[ID].FlagFrameFull[FrameType];
	}
}

uint8_t CAN_Send_Application(CANBufferHandleStruct *AppBuffer,
		CANConfigIDTxtypedef *pStID, uint8_t *Data, uint8_t DataLength) {
	return CAN_Send_Network_Packet(AppBuffer, Data, DataLength, pStID);
}

uint8_t CAN_Send_Network_Packet(CANBufferHandleStruct *TxBuffer, uint8_t *Data,
		uint8_t DataLength, CANConfigIDTxtypedef *pStID) {
	TxBuffer->PacketDataLength = DataLength + 2;
	TxBuffer->CRCValue = crc_8(Data, DataLength);
	TxBuffer->Buffer_Index = DataLength;
	if (TxBuffer->PacketDataLength % 8 == 0) {
		TxBuffer->NumberOfFrame = (TxBuffer->PacketDataLength / 8);
	} else {
		TxBuffer->NumberOfFrame = (TxBuffer->PacketDataLength / 8) + 1;
	}
	memcpy(TxBuffer->NetworkBuffer, Data, DataLength);
	TxBuffer->NetworkBuffer[TxBuffer->Buffer_Index] =
			TxBuffer->PacketDataLength;
	TxBuffer->NetworkBuffer[TxBuffer->Buffer_Index + 1] = TxBuffer->CRCValue;
	TxBuffer->Buffer_Index = 0;
	return CAN_Send_DataLink_Separate(TxBuffer, Data, pStID);
}
uint8_t CAN_Send_DataLink_Separate(CANBufferHandleStruct *TxBuffer,
		uint8_t *Data, CANConfigIDTxtypedef *pStID) {
	uint8_t PacketLength = TxBuffer->PacketDataLength;
	uint8_t NumberOfFrame = TxBuffer->NumberOfFrame;
	TxBuffer->Buffer[NumberOfFrame - 1][6] = PacketLength;
	TxBuffer->Buffer[NumberOfFrame - 1][7] = TxBuffer->CRCValue;
	for (int i = 0; i < NumberOfFrame; i++) {
		for (TxBuffer->Buffer_Index = 0; TxBuffer->Buffer_Index < 8;
				TxBuffer->Buffer_Index++) {
			TxBuffer->Buffer[i][TxBuffer->Buffer_Index] =
					TxBuffer->NetworkBuffer[i * 8 + TxBuffer->Buffer_Index];
			PacketLength--;
			if (PacketLength == 2) {
				break;
			}
		}
		if (PacketLength == 2) {
			break;
		}
	}
	TxBuffer->Buffer_Index = 0;
	return CAN_Send_Physical_Send(TxBuffer, Data, pStID);
}
uint8_t CAN_Send_Physical_Send(CANBufferHandleStruct *TxBuffer, uint8_t *Data,
		CANConfigIDTxtypedef *pIDtype) {
	twai_message_t Txheader;
	uint8_t Message_ID = pIDtype->MessageType;
	uint8_t Sender_ID = pIDtype->SenderID;
	uint8_t FrameType = TxBuffer->FrameType_Index;
	uint8_t NumberOfFrame = TxBuffer->NumberOfFrame;
	uint16_t StdId = 0x00;

	StdId |= Message_ID;
	StdId = (StdId << 4) | Sender_ID;
	TxBuffer->SenderID = StdId;
	StdId = (StdId << 3) | TxBuffer->FrameType_Index;
	Txheader.data_length_code = 8;
	Txheader.flags = TWAI_MSG_FLAG_NONE;

	for (int8_t i = NumberOfFrame - 1; i >= 0; i--) {
		Txheader.identifier = StdId;
		Txheader.data[0]=TxBuffer->Buffer[i][0];
		Txheader.data[1]=TxBuffer->Buffer[i][1];
		Txheader.data[2]=TxBuffer->Buffer[i][2];
		Txheader.data[3]=TxBuffer->Buffer[i][3];
		Txheader.data[4]=TxBuffer->Buffer[i][4];
		Txheader.data[5]=TxBuffer->Buffer[i][5];
		Txheader.data[6]=TxBuffer->Buffer[i][6];
		Txheader.data[7]=TxBuffer->Buffer[i][7];
		if (twai_transmit(&Txheader,
				portMAX_DELAY) != ESP_OK) {
		}
		StdId = StdId >> 3;
		FrameType++;
		StdId = (StdId << 3) | FrameType;

	}
	return ESP_OK;
}

void CAN_Recieve_Physical_FIFO0(twai_message_t *RxHeader) {

	if (twai_receive(RxHeader,portMAX_DELAY) != ESP_OK) {
	}
}

uint8_t CAN_Receive_DataLink(FlagFrameHandle *FlagHandle,
		CANBufferHandleStruct *RxBuffer, FlagRecNotification *FlagNotiHandle) {
	twai_message_t RxHeader; 
	*FlagNotiHandle = REC_DATA;
	uint16_t StdID = 0;
	uint8_t ID = 0;
	CAN_Recieve_Physical_FIFO0(&RxHeader);
	//CAN_Recieve_Physical_FIFO1(&RxHeader,DataPhysical);
	StdID = RxHeader.identifier;
	ID = (StdID >> 3) & 15;
	RxBuffer->RecvID = ID;
	RxBuffer->NodeHandle[ID].FrameType = StdID & 7; // get frame type store into Rxbuffer struct with Node ID manage frame type
	if (RxBuffer->NodeHandle[ID].FrameType == SET_UP_FRAME
			&& RxBuffer->NodeHandle[ID].DuplicateFrame != 1) { // check if frame type = SET_UP_FRAME
		RxBuffer->NodeHandle[ID].DuplicateFrame = 1; // check send multiple SET_UP_frame
		RxBuffer->NodeHandle[ID].PacketLength = RxHeader.data[6];
		RxBuffer->NodeHandle[ID].CRCValue = RxHeader.data[7];
		if (RxBuffer->NodeHandle[ID].PacketLength % 8 == 0) {
			RxBuffer->NodeHandle[ID].NumberOfFrame =
					(RxBuffer->NodeHandle[ID].PacketLength / 8);
		} else {
			RxBuffer->NodeHandle[ID].NumberOfFrame =
					(RxBuffer->NodeHandle[ID].PacketLength / 8) + 1;
		}
	} else {
		if (RxBuffer->NodeHandle[ID].FrameType == SET_UP_FRAME) {
			*FlagNotiHandle = REC_FRAMEDATA_ERROR;
			FlagHandle->FlagID[ID].FrameError[RxBuffer->NodeHandle[ID].FrameType] =
					1;
			// CAN_Receive_Error_Handle(FlagNotiHandle, FlagHandle);
		}
	}
	CAN_ProcessRxBuffer(FlagHandle, ID, RxBuffer, RxHeader.data, FlagNotiHandle);
	return ESP_OK;
}
uint8_t CAN_Receive_Network(CANBufferHandleStruct *NetBuffer,
		FlagFrameHandle *NetworkFlag, FlagRecNotification *FlagNotiHandle) {
	CAN_Receive_DataLink(NetworkFlag, NetBuffer, FlagNotiHandle);
	uint8_t FrameLength = 0;
	uint8_t FrameType = 0;
	uint8_t NetBufferIndex = 0;
	uint8_t DataLength = 0;
	uint8_t CRCValue = 0;
	uint8_t *NetData;
	FrameLength = NetBuffer->NodeHandle[NetBuffer->RecvID].NumberOfFrame;
	uint8_t NumberofFrame = FrameLength;
	FrameType = NetBuffer->NodeHandle[NetBuffer->RecvID].FrameType;
	if (*FlagNotiHandle == REC_FRAMEDATA_SUCCESS) {
		for (; FrameLength > 0; FrameLength--) {
			memcpy(NetBuffer->Buffer[NetBufferIndex],
					NetBuffer->NodeHandle[NetBuffer->RecvID].NodeBuffer[FrameType],
					CAN_MAX_DATA);
			NetBufferIndex++;
			FrameType--;
		}
		DataLength = NetBuffer->NodeHandle[NetBuffer->RecvID].PacketLength - 2;
		NetData = (uint8_t*) malloc(DataLength * sizeof(uint8_t));
		for (NetBufferIndex = 0; NetBufferIndex <= NumberofFrame;
				NetBufferIndex++) {
			for (int j = 0; j < 8; j++) {
				NetData[NetBufferIndex * 8 + j] =
						NetBuffer->Buffer[NetBufferIndex][j];
			}
		}
		CRCValue = crc_8(NetData, DataLength);
		if (CRCValue == NetBuffer->NodeHandle[NetBuffer->RecvID].CRCValue) {
			*FlagNotiHandle = REC_PACKET_SUCCESS;
			memcpy(NetBuffer->NetworkBuffer, NetData, DataLength);
		} else {
			*FlagNotiHandle = REC_PACKET_ERROR;
			// CAN_Receive_Error_Handle(FlagNotiHandle, NetworkFlag);
		}
		free(NetData);
	}
	return ESP_OK;
}

uint8_t CAN_Receive_Application(CANBufferHandleStruct *AppBuffer, uint8_t *Data,
		FlagFrameHandle *FlagFrame, FlagRecNotification *FlagNotification) {
	uint8_t AppDataLength =
			AppBuffer->NodeHandle[AppBuffer->RecvID].PacketLength - 2;
	CAN_Receive_Network(AppBuffer, FlagFrame, FlagNotification);
	if (*FlagNotification == REC_PACKET_SUCCESS) {
		memcpy(Data, AppBuffer->NetworkBuffer, AppDataLength);
		*FlagNotification = REC_SUCCESS;
	}
	return ESP_OK;
}