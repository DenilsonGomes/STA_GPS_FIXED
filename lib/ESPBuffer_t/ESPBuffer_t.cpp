#include "ESPBuffer.h"

template<typename T, uint16_t S>
bool Buffer<T, S>::readHeadTail(){
	File configFile = SPIFFS.open("/bufferConfig.txt", "r");
	if (!configFile){
		Serial << F("[Erro] Failed to open bufferConfig.conf") << endl;
		return false;
	}
	configFile.seek(0, SeekSet);

	uint16_t *pConf = &head;
	uint8_t arrayConf[sizeof(uint16_t)];

	for(int i=0; i<sizeof(uint16_t); i++){
		arrayConf[i]=configFile.read();
	}
	memcpy(pConf, (uint16_t *)arrayConf, sizeof(uint16_t));

	pConf = &tail;

	for(int i=0; i<sizeof(uint16_t); i++){
		arrayConf[i]=configFile.read();
	}
	memcpy(pConf, (uint16_t *)arrayConf, sizeof(uint16_t));

	configFile.close();

	return true;
}

template<typename T, uint16_t S>
bool Buffer<T, S>::writeHeadTail(){
	File configFile = SPIFFS.open("/bufferConfig.txt", "r+");//Verificar se o arquivo foi aberto corretamente
	//Atualiza valores de head e tail em /bufferConfig.txt
	configFile.seek(0, SeekSet);

	uint8_t * data = reinterpret_cast<uint8_t*>(&head);
	configFile.write(data, sizeof(data));
	data = reinterpret_cast<uint8_t*>(&tail);
	configFile.write(data, sizeof(data));

	configFile.close();
}

template<typename T, uint16_t S>
bool Buffer<T, S>::init()
{
	int i;

	if(!SPIFFS.exists("/buffer.conf"))
	{
		File initFile = SPIFFS.open("/buffer.conf", "w+");//Testar se necessita ser declarado em um escopo superior

		if (!initFile){
			Serial << F("[Erro] Failed to creat buffer.conf") << endl;
			return false;
		}

		uint8_t dd=0;//verificar a necessidade de escrever esse arquivo dessa forma

		for(i=0; i<(S*sizeof(T)); i++){
			initFile.write(&dd, 1);
		}

		initFile.close();
	}
	if(!SPIFFS.exists("/bufferConfig.txt"))
	{
		File configFile = SPIFFS.open("/bufferConfig.txt", "w+");

		if (!configFile){
			Serial << F("[Erro] Failed to creat bufferConfig.conf") << endl;
			return false;
		}

		uint8_t * data = reinterpret_cast<uint8_t*>(&head);

		configFile.write(data, sizeof(data));
		data = reinterpret_cast<uint8_t*>(&tail);

		configFile.write(data, sizeof(data));
		configFile.close();

		return true;
	}

	return readHeadTail();
}

template<typename T, uint16_t S>
void Buffer<T, S>::push(T value)
{
	File pushFile = SPIFFS.open("/buffer.conf", "r+");

	readHeadTail();

	//Escreve o dado no buffer
	pushFile.seek(tail, SeekSet);

	uint8_t * data = reinterpret_cast<uint8_t*>(&value);
	pushFile.write(data, sizeof(T));

	tail = (tail+sizeof(T))%(S*sizeof(T)); //calc endereço

	if(tail == head){
		head = (head+sizeof(T))%(S*sizeof(T));
	}

	writeHeadTail();

	pushFile.close();
}


/*
(T *value, *foo)
*/
template<typename T, uint16_t S>
bool Buffer<T, S>::rescue(T *value)
{
	uint8_t ret = 0;

	File rescueFile = SPIFFS.open("/buffer.conf", "r");

	readHeadTail();

	//Tenta resgatar o dado//isEmpty
	rescueFile.seek(head, SeekSet);

	T *PTdado = value;//&auxSensor;
	uint8_t arrayFile[sizeof(T)];

	for(int i=0; i<sizeof(T); i++){
		arrayFile[i]=rescueFile.read();
	}
	memcpy(PTdado, (T *)arrayFile, sizeof(T));
	//Testa se o resgate foi bem sucedido em foo
	/*
	if(!foo(value)){
		rescueFile.close();
		configFile.close();
		return false;
	}
	*/
	char * pub = toJSON(value);
	while(!estacao.publish(pub)){
		ret++;
		delay(20);//aplicar delay inteligente
		if(ret > MAXRETRIES){
			rescueFile.close();
			configFile.close();

			return false;
		}
	}

	head = (head+sizeof(T))%(S*sizeof(T));

	writeHeadTail();

	rescueFile.close();

	return true;
}

template<typename T, uint16_t S>
bool Buffer<T, S>::isEmpty()
{
	readHeadTail();

	if(head == tail)
		return true;
	return false;
}