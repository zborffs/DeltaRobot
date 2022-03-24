#ifndef DELTA_COMPUTERVISION_HPP
#define DELTA_COMPUTERVISION_HPP

#include <opencv2/opencv.hpp>

/**
 * This class computes the FEN from an image of a chess position, by segmenting the image into 64 squares and using a
 * pre-trained classifier to classify the piece on each square.
 */
class PositionRecognizer {
public:
    /**
     * Given an image of a chess position, returns the FEN of the position
     * @param image (dim: ())
     * @return FEN string
     */
    std::string_view chess_position(cv::Mat& image);

private:
    /**
     * Given an image of a chess board, this function segments the image into 64 images of each square.
     * @param image
     */
    std::array<cv::Mat, 64> segment_image(cv::Mat& image);

    /**
     * given an image of a square, this function returns the char of the piece on the square in the image
     * @param image (dim: ()) image of a single square on the board
     * @return character representing the piece on the square (P, p, R, r, N, n, B, b, Q, q, K, k, ' '); upper-case is W
     */
    char classify_piece(cv::Mat& image);
};

#endif //DELTA_COMPUTERVISION_HPP
