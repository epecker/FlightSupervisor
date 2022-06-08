#include <string>

#include <boost/interprocess/shared_memory_object.hpp>

#include <cadmium/modeling/dynamic_model.hpp>

using namespace std;
using namespace boost::interprocess;

class Callback_Tester {
    private:
        string name;
        char *mem;
        int current;
        int prev;

    public:
        Callback_Tester(string name) {
            this.name = name;
            shared_memory_object shm(open_or_create, this.name.cstr(), read_write);
        
            //Set size
            shm.truncate(sizeof(int));

            //Map the whole shared memory in this process
            mapped_region region(shm, read_write);

            //Write all the memory to 1
            std::memset(region.get_address(), 1, region.get_size());
            
            mem = static_cast<char*>(region.get_address());
            current = *mem;
            prev = current;
        }

        void monitor(AsyncEventSubject sub) {
            while (1) {
                current = *mem;
                if (current != prev) {
                    sub.notify();
                }
                prev = current;
            }
        }

}