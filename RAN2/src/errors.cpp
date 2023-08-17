#include "../include/errors.hpp"

operation_status operation_status_init(operation_module_code module, operation_result result, operation_result_code code){
    operation_status status;
    status.module = module;
    status.result = result;
    status.code = code;
    return status;
}