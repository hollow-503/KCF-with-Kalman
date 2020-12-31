//
// Created by hollow on 2020/12/15.
//

#include "util/util.h"
#include "process/process.h"

using namespace std;

int main() {
    util::LoadParam();
    process Process;
    Process.run();
    return 0;
}
