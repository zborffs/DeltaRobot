#include "position_recognizer.hpp"

std::string_view PositionRecognizer::chess_position(cv::Mat &image) {
    return "asdf";
}

std::array<cv::Mat, 64> PositionRecognizer::segment_image(cv::Mat &image) {
    return {};
}

char PositionRecognizer::classify_piece(cv::Mat &image) {
    return ' ';
}
