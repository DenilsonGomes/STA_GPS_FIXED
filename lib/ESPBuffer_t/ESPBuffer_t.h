template<typename T, uint16_t S> class Buffer{
	uint16_t head=0;
	uint16_t tail=0;
	public:
		bool readHeadTail();
		bool init();
		void push(T value);
		bool rescue(T *value);
		bool isEmpty();
};