#include "../include/aura/qr_handler.h"

QRHandler::QRHandler(const std_msgs::StringConstPtr &qr_msg) {
    QRHandler::qr_code = new std::string(qr_msg->data);
}

void QRHandler::compute() {
    //todo Only for 4 direction QR codes
    //check all QR codes has 4 direction
    QRHandler::goals = (char *) malloc(4 * sizeof(char));
    memset(QRHandler::goals, -1, 4 * sizeof(char));
    int index = 0;
    for (char c : *(QRHandler::qr_code)) {
        if (index > 3) {
            QRHandler::goal_id = c - '0';
            break;
        }
        if (c == ',') {
            index++;
        } else {
            *(QRHandler::goals + index) = c;
        }
    }
    QRHandler::goal = QRHandler::goal_id != 0;
}

bool QRHandler::is_goal() {
    return QRHandler::goal;
}


char QRHandler::get_target(int target) {
    if (target > 3) {
        return -1;
    }
    return *(QRHandler::goals + target);
}

int QRHandler::get_goal() {
    if (QRHandler::is_goal()) {
        return QRHandler::goal_id;
    } else {
        return -1;
    }
}

std::string QRHandler::get_all_targets() {
    std::stringstream target_stream;
    for (int i = 0; i < 4; i++) {
        target_stream << QRHandler::get_target(i) << " ";
    }
    std::string targets = target_stream.str();
    return targets.substr(0, targets.length() - 1);
}
