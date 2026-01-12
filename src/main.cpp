#include <filesystem>
#include <libimages/draw.h>
#include <libimages/algorithms/blur.h>
#include <libimages/algorithms/downsample.h>
#include <libimages/algorithms/grayscale.h>
#include <libimages/algorithms/threshold_masking.h>
#include <libimages/algorithms/morphology.h>
#include <libimages/algorithms/split_into_parts.h>
#include <libimages/algorithms/extract_contour.h>
#include <libimages/algorithms/simplify_contours.h>

#include <libbase/stats.h>
#include <libbase/timer.h>
#include <libbase/fast_random.h>
#include <libbase/runtime_assert.h>
#include <libbase/configure_working_directory.h>
#include <libimages/debug_io.h>
#include <libimages/image.h>
#include <libimages/image_io.h>

#include <iostream>
#include <unordered_map>

#include "sides_comparison_utils.h"
#include "puzzle_assembly.h"

int main() {
    try {
        configureWorkingDirectory();

        // это список картинок которые вы хотите обработать при запуске
        // сначала тестироваться лучше всего на маленькой картинке (первая в списке)
        // но если все работает и хочется дополнительно проверить алгоритм,
        // то раскомментируйте остальные строчки и проверьте алгоритм и на них
        // отладочная визуализация теперь сохраняется не напрямую в debug,
        // а в подпапке вида debug/00_photo_six_parts_downscaled_x4 (название соответствует картинке)
        std::vector<std::string> to_process = {
            "00_photo_six_parts_downscaled_x4",
            // "00_photo_six_parts",
            // "01_eight_parts",
            // "02_eight_parts_shuffled",
            // "03_eight_parts_shuffled2",
        };

        // создание визуализации каждой пары сопоставлений занимает большое время, поэтому оставим этот выключатель на будущее
        // когда нужен просто результат без анализа - можно будет выключить
        bool draw_sides_matching_plots = false;

        Timer all_images_t;
        for (const std::string &image_name: to_process) {
            Timer total_t;
            Timer t;

            std::string debug_dir = "debug/" + image_name + "/";
            // удаляем папку чтобы не анализировать случайно старые визуализации
            std::filesystem::remove_all(debug_dir);

            image8u image = load_image("data/" + image_name + ".jpg");
            auto [w, h, c] = image.size();
            rassert(c == 3, 237045347618912, image.channels());
            std::cout << "image loaded in " << t.elapsed() << " sec" << std::endl;
            debug_io::dump_image(debug_dir + "00_input.jpg", image);

            image32f grayscale = to_grayscale_float(image);
            rassert(grayscale.channels() == 1, 2317812937193);
            rassert(grayscale.width() == w && grayscale.height() == h, 7892137419283791);
            debug_io::dump_image(debug_dir + "01_grayscale.jpg", grayscale);

            std::vector<float> intensities_on_border;
            for (int j = 0; j < h; ++j) {
                for (int i = 0; i < w; ++i) {
                    // пропускаем все пиксели кроме границы изображения
                    if (i != 0 && i != w - 1 && j != 0 && j != h - 1)
                        continue;
                    intensities_on_border.push_back(grayscale(j, i));
                }
            }
            // DONE: какой инвариант мы можем проверить про размер intensities_on_border.size()? чем он должен быть равен?
            rassert(intensities_on_border.size() == 2 * w + 2 * h - 4, 7283197129381312);
            std::cout << "intensities on border: " << stats::summaryStats(intensities_on_border) << std::endl;

            // DONE: найдем порог разделяющий яркость на фон и объект - background_threshold
            double background_threshold = 1.5 * stats::percentile(intensities_on_border, 90);
            std::cout << "background threshold=" << background_threshold << std::endl;

            // DONE: построим маску объект-фон + сохраним визуализацию на диск + выведем в лог процент пикселей на фоне
            image8u is_foreground_mask = threshold_masking(grayscale, background_threshold);
            double is_foreground_sum = stats::sum(is_foreground_mask.toVector());
            std::cout << "thresholded background: " << stats::toPercent(w * h - is_foreground_sum / 255.0, 1.0 * w * h) << std::endl;
            debug_io::dump_image(debug_dir + "02_is_foreground_mask.png", is_foreground_mask);

            t.restart();
            // DONE: сделаем маску более гладкой и точной через Морфологию
            // DONE: сначала попробуем dilation + erosion, все ли хорошо поулчилось? нет ли выбросов?
            int strength = 6;

            const bool with_openmp = true;
            image8u dilated_mask = morphology::dilate(is_foreground_mask, strength, with_openmp);
            image8u dilated_eroded_mask = morphology::erode(dilated_mask, strength, with_openmp);
            image8u dilated_eroded_eroded_mask = morphology::erode(dilated_eroded_mask, strength, with_openmp);
            image8u dilated_eroded_eroded_dilated_mask = morphology::dilate(dilated_eroded_eroded_mask, strength, with_openmp);

            // добавляем эрозию на один-два шага чтобы при взятии цветов для описания сторон - не брать случайно черные цвета с фона
            // эта проблема особенно ярко заметна на белых сторонах - там много черных вкраплений
            // и хорошо видно что график вместо того чтобы быть в высоких около-255 значениях - часто скакал вниз
            dilated_eroded_eroded_dilated_mask = morphology::erode(dilated_eroded_eroded_dilated_mask, 2, with_openmp);

            std::cout << "full morphology in " << t.elapsed() << " sec" << std::endl;

            // DONE 1 посмотрите на RGB графики тех сторон у которых нет и не может быть соседей, то есть у белых полос
            // разумно ли они выглядят? с чем это может быть связано? как это исправить?
            debug_io::dump_image(debug_dir + "03_is_foreground_dilated.png", dilated_mask);
            debug_io::dump_image(debug_dir + "04_is_foreground_dilated_eroded.png", dilated_eroded_mask);
            debug_io::dump_image(debug_dir + "05_is_foreground_dilated_eroded_eroded.png", dilated_eroded_eroded_mask);
            debug_io::dump_image(debug_dir + "06_is_foreground_dilated_eroded_eroded_dilated.png", dilated_eroded_eroded_dilated_mask);

            is_foreground_mask = dilated_eroded_eroded_dilated_mask;
            auto [objOffsets, objImages, objMasks] = splitObjects(image, is_foreground_mask);
            int objects_count = objImages.size();
            std::cout << objects_count << " objects extracted" << std::endl;
            rassert(objects_count == 6 || objects_count == 8, 237189371298, objects_count);

            // визуализируем цветами компоненты связности - один объект - один цвет
            image32i image_with_object_indices(image.width(), image.height(), 1);
            for (int obj = 0; obj < objects_count; ++obj) {
                // это отступ - координата верхнего левого угла объекта на оригинальной картинке
                point2i offset = objOffsets[obj];

                // это маска объекта
                image8u mask = objMasks[obj];

                for (int j = 0; j < mask.height(); ++j) {
                    for (int i = 0; i < mask.width(); ++i) {
                        // если объект в своей маске отмечен как "тут объект"
                        if (mask(j, i) == 255) {
                            // то рассчитываем координаты этого пикселя в оригинальной картинке и пишем туда наш номер (индексация с 1)
                            int global_i = offset.x + i;
                            int global_j = offset.y + j;
                            image_with_object_indices(global_j, global_i) = obj + 1;
                        }
                    }
                }
            }
            debug_io::dump_image(debug_dir + "07_colorized_objects.jpg", debug_io::colorize_labels(image_with_object_indices, 0));

            std::vector<std::vector<std::vector<point2i>>> objSides(objects_count);
            std::vector<std::vector<point2i>> objCorners(objects_count);
            for (int obj = 0; obj < objects_count; ++obj) {
                std::string obj_debug_dir = debug_dir + "objects/object" + std::to_string(obj) + "/";

                debug_io::dump_image(obj_debug_dir + "01_image.jpg", objImages[obj]);
                debug_io::dump_image(obj_debug_dir + "02_mask.jpg", objMasks[obj]);

                // DONE реализуйте построение маски контура-периметра, нажмите Ctrl+Click на buildContourMask:
                image8u objContourMask = buildContourMask(objMasks[obj]);

                debug_io::dump_image(obj_debug_dir + "03_mask_contour.jpg", objContourMask);

                std::vector<point2i> contour = extractContour(objContourMask);

                // сделаем черную картинку чтобы визуализировать контур на ней
                image32f contour_visualization(objImages[obj].width(), objImages[obj].height(), 1);

                // нарисуем на ней контур
                for (int i = 0; i < contour.size(); ++i) {
                    point2i pixel = contour[i];
                    // сделаем цвет тем ярче - чем дальше пиксель в контуре (чтобы проверить что он по часовой стрелке)
                    drawPoint(contour_visualization, pixel, color32f(i * 255.0f / contour.size()));
                }

                debug_io::dump_image(obj_debug_dir + "04_mask_contour_clockwise.jpg", contour_visualization);

                // у нас теперь есть перечень пикселей на контуре объекта
                // DONE реализуйте определение в этом контуре 4 вершин-углов и нарисуйте их на картинке, нажмите Ctrl+Click на simplifyContour:
                std::vector<point2i> corners = simplifyContour(contour, 4);
                objCorners[obj] = corners;
                rassert(corners.size() == 4, 32174819274812);

                // сделаем черную картинку чтобы визуализировать вершины-углы на ней
                image32f corners_visualization(objImages[obj].width(), objImages[obj].height(), 1);
                for (point2i corner: corners) {
                    drawPoint(corners_visualization, corner, color32f(255.0f), 10);
                }
                debug_io::dump_image(obj_debug_dir + "05_corners_visualization.jpg", corners_visualization);

                // теперь извлечем стороны объекта
                std::vector<std::vector<point2i>> sides = splitContourByCorners(contour, corners);
                rassert(sides.size() == 4, 237897832141);

                // визуализируем каждую сторону объекта отдельным цветом:
                image8u sides_visualization(objImages[obj].width(), objImages[obj].height(), 3);
                FastRandom r(2391);
                for (int i = 0; i < sides.size(); ++i) {
                    color8u random_color = {(uint8_t) r.nextInt(0, 255), (uint8_t) r.nextInt(0, 255), (uint8_t) r.nextInt(0, 255)};
                    color8u side_color = random_color;
                    drawPoints(sides_visualization, sides[i], side_color);
                }
                debug_io::dump_image(obj_debug_dir + "06_sides.jpg", sides_visualization);

                objSides[obj] = sides;
            }

            // в этом векторе мы будем хранить сопоставления:
            // MatchedSide.objB - индекс сопоставленного объекта-кусочка пазла
            // MatchedSide.sideB - индекс сопоставленной стороны сопоставленного кусочка
            // MatchedSide.differenceBest - насколько отличаются цвета (по нашей метрике, 0 - совпадают идеально)
            // MatchedSide.differenceSecondBest - насколько отличаются цвета со второй по лучшевизне сопоставленной стороной
            //        (нужно для анализа "насколько наша метрика уверенно отличила правильный ответ от ложного")
            // если сопоставления не нашлось: -1 -1 -1
            std::vector<std::vector<MatchedSide>> objMatchedSides(objects_count);

            // теперь будем сопоставлять каждую сторону объекта с каждой другой стороной другого объекта
            std::cout << "matching sides with each other" << std::endl;
            // перебираем объект А и его сторону для которой мы будем искать сопоставление
            for (int objA = 0; objA < objects_count; ++objA) {
                std::string obj_debug_dir = debug_dir + "objects/object" + std::to_string(objA) + "/";
                objMatchedSides[objA].resize(objSides[objA].size());
                rassert(objMatchedSides[objA][0].differenceBest == -1, 23423431);
                for (int sideA = 0; sideA < objSides[objA].size(); ++sideA) {
                    // мы знаем из каких пикселей брать цвета для этих точек
                    const std::vector<point2i> pixelsA = objSides[objA][sideA];
                    // извлекаем цвета пикселей из картинки объекта A
                    const std::vector<color8u> colorsA = extractColors(objImages[objA], pixelsA);
                    const int channels = objImages[objA].channels();

                    if (isMostlyWhite(colorsA)) {
                        // пропускаем стороны которые почти полностью белые - это край всего изображения
                        // ждя них нет соседних кусочков паззла, значит не нужно их сопоставлять (в результате сопоставляя с кем-то случайным)
                        continue;
                    }

                    // перебираем другой объект B и его сторону с которой мы хотим попробовать себя сравнить
                    for (int objB = 0; objB < objects_count; ++objB) {
                        if (objA == objB)
                            continue;
                        for (int sideB = 0; sideB < objSides[objB].size(); ++sideB) {
                            // мы знаем из каких пикселей брать цвета для точек второй стороны B
                            std::vector<point2i> pixelsB = objSides[objB][sideB];
                            // разворачиваем пиксели стороны в обратном порядке, ведь мы хотим как zip-молнию
                            // сравнить их пиксель за пикселем, каждый из этих списков пикселей стороны - по часовой стрелке
                            // значит они как борящиеся друг против друга шестеренки трутся и расходятся в противоположных направлениях
                            // поэтому нужно их сориентировать инвертировав порядок одного из них
                            std::reverse(pixelsB.begin(), pixelsB.end()); // да, эту строку нужно раскомментировать
                            // извлекаем цвета пикселей из картинки объекта A
                            const std::vector<color8u> colorsB = extractColors(objImages[objB], pixelsB);
                            rassert(channels == objImages[objB].channels(), 34712839741231);

                            if (isMostlyWhite(colorsB)) {
                                // пропускаем стороны которые почти полностью белые - это край всего изображения
                                // ждя них нет соседних кусочков паззла, значит не нужно их сопоставлять (в результате сопоставляя с кем-то случайным)
                                continue;
                            }

                            // чтобы удобно было сравнивать - нужно чтобы эти две стороны были выравнены по длине
                            int n = std::min(colorsA.size(), colorsB.size());
                            // DONE 2 посмотрите на графики и подумайте, может имеет смысл как-то воздействовать на снятые с границы цвета?
                            // например сгладить? если решите попробовать - воспользуйтесь готовой функцией blur(std::vector<color8u> colors, float strength)
                            float blur_strength = 4.0f;
                            std::vector<color8u> a = downsample(blur(colorsA, blur_strength), n);
                            std::vector<color8u> b = downsample(blur(colorsB, blur_strength), n);
                            rassert(a.size() == n && b.size() == n, 2378192321);

                            // теперь давайте в каждой паре пикселей оценим насколько сильно они отличаются
                            std::vector<float> differences(n);
                            for (int i = 0; i < n; ++i) {
                                float d = 0;
                                color8u colA = a[i];
                                color8u colB = b[i];
                                // DONE 3 реализуйте какую-то метрику сравнивающую насколько эти два цвета colA и colB отличаются
                                for (int c = 0; c < channels; ++c) {
                                    uint8_t colAChannelIntensity = colA(c);
                                    uint8_t colBChannelIntensity = colB(c);
                                    d += std::abs((int) colAChannelIntensity - colBChannelIntensity);
                                }
                                differences[i] = d;
                            }
                            for (int i = 0; i < n; ++i) {
                                rassert(differences[i] >= 0.0f, 32423415214, differences[i]);
                            }

                            // DONE 4 и наконец финальный вердикт - насколько сильно отличаются эти две стороны? например это может быть медиана попиксельных разниц
                            // (не забудьте про stats::median, stats:sum, stats::percentile)
                            float total_difference = stats::median(differences); // это совсем простой вариант чтобы код просто компилировался - тут мы берем разницу первого попавшегося одного пикселя

                            float previous_best = objMatchedSides[objA][sideA].differenceBest;
                            if (previous_best == -1 || total_difference <= previous_best) {
                                // если раньше сопоставления еще не было вовсе (-1)
                                // если если наше сопоставление лучше (наша разница меньше старой)
                                // то сохраняем текущее сопоставление как пока что лучший ответ (старый ответ становится вторым по лучшевизне)
                                objMatchedSides[objA][sideA] = {objB, sideB, total_difference, previous_best};
                            }

                            if (draw_sides_matching_plots) {
                                // сделаем небольшой предпросмотр обоих объектов с отмеченными сторонами
                                int preview_image_width = n;
                                int preview_image_height = n;

                                int colors_rgb_line_height = 10;
                                int separator_line_height = 3;
                                int graph_height = 100;
                                // визуализируем наложение этих двух сторон
                                image8u ab_visualization(n + n, std::max(2 * preview_image_height,  2 * colors_rgb_line_height + 4 * separator_line_height + 2 * graph_height + graph_height), 3);

                                // сначала нарисуем объект A + на нем отмеченная сторона A
                                point2i offset = {0, 0}; // это точка отступа - где находится угол следующего рисуемого объекта
                                image8u previewA = objImages[objA];
                                drawPoints(previewA, objSides[objA][sideA], color8u(255, 0, 0), 5);
                                previewA = downsample(blur(previewA, previewA.width() / preview_image_width), preview_image_width, preview_image_height);
                                drawImage(ab_visualization, previewA, offset);
                                offset.y += preview_image_height; // смещаем отступ на высоту нарисованной картинки

                                // затем объект B + на нем отмеченная сторона B
                                image8u previewB = objImages[objB];
                                drawPoints(previewB, objSides[objB][sideB], color8u(255, 0, 0), 5);
                                previewB = downsample(blur(previewB, previewB.width() / preview_image_width), preview_image_width, preview_image_height);
                                drawImage(ab_visualization, previewB, offset);
                                offset.y += preview_image_height;

                                // графики рисуем в правой части картинки
                                offset = {preview_image_width, 0};

                                // сначала наложим сами цвета обеих сторон
                                drawRGBLine(ab_visualization, a, offset, colors_rgb_line_height);
                                offset.y += colors_rgb_line_height;
                                drawRGBLine(ab_visualization, b, offset, colors_rgb_line_height);
                                offset.y += colors_rgb_line_height;

                                std::vector<color8u> separator_line_colors(n, color8u(0, 255, 0));

                                // затем построим графики яркости этих сторон - красным цветом график яркости RED канала, зеленым и синим - GREEN/BLUE соответственно
                                drawRGBLine(ab_visualization, separator_line_colors, offset, separator_line_height);
                                offset.y += separator_line_height;
                                drawGraph(ab_visualization, a, offset, graph_height);
                                offset.y += graph_height;
                                drawRGBLine(ab_visualization, separator_line_colors, offset, separator_line_height);
                                offset.y += separator_line_height;
                                drawGraph(ab_visualization, b, offset, graph_height);
                                offset.y += graph_height;
                                drawRGBLine(ab_visualization, separator_line_colors, offset, separator_line_height);
                                offset.y += separator_line_height;

                                // затем визуализируем графиком нашу метрику отличия
                                float normalization_value = 100.0f; // график имеет шкалу от 0 до normalization_value
                                drawGraph(ab_visualization, differences, offset, graph_height, normalization_value);
                                offset.y += graph_height;
                                drawRGBLine(ab_visualization, separator_line_colors, offset, separator_line_height);
                                offset.y += separator_line_height;

                                // заметьте что мы специально в начале файла пишем diff (еще и дополненный нулями)
                                // благодаря этому мы прямо в списке файлов будем видеть лучшее и худшее сопоставление
                                debug_io::dump_image(obj_debug_dir + "side" + std::to_string(sideA)
                                    + "/diff=" + pad(total_difference, 5) + "_with_object" + std::to_string(objB) + "_side" + std::to_string(sideB) + ".png",
                                    ab_visualization);
                            }
                        }
                    }
                }
            }

            std::unordered_map<std::string, std::vector<std::vector<MatchedSide>>> correct_matches;
            {
                // захаркодим ответы для маленькой картинки, чтобы всегда сразу видеть сколько ответов у нас верно,
                // а сколько - нет
                // благодаря детерминизму алгоритма (у нас даже все FastRandom ведут себя из раза в раз - ОДИНАКОВО)
                // от запуска к запуску все четко повторяется, включая нумерацию объектов и сторон
                // поэтому возможно вручную фиксировать правильный ответ
                std::vector<std::vector<MatchedSide>> answers(objects_count);
                for (int obj = 0; obj < objects_count; ++obj) {
                    answers[obj].resize(objSides[obj].size());
                }
                answers[0][0] = MatchedSide(1, 2, 239, 239);
                answers[0][1] = MatchedSide(3, 3, 239, 239);
                answers[1][0] = MatchedSide(2, 3, 239, 239);
                answers[1][1] = MatchedSide(5, 3, 239, 239);
                answers[1][2] = MatchedSide(0, 0, 239, 239);
                answers[2][2] = MatchedSide(4, 3, 239, 239);
                answers[2][3] = MatchedSide(1, 0, 239, 239);
                answers[3][0] = MatchedSide(5, 2, 239, 239);
                answers[3][3] = MatchedSide(0, 1, 239, 239);
                answers[4][2] = MatchedSide(5, 0, 239, 239);
                answers[4][3] = MatchedSide(2, 2, 239, 239);
                answers[5][0] = MatchedSide(4, 2, 239, 239);
                answers[5][2] = MatchedSide(3, 0, 239, 239);
                answers[5][3] = MatchedSide(1, 1, 239, 239);

                correct_matches["00_photo_six_parts_downscaled_x4"] = answers;
            }

            {
                // нарисуем отрезками сопоставления между сторонами
                int segment_thickness = 5;
                image8u segments_between_matched_sides = image;
                FastRandom r(2391);
                int correct_matches_count = 0;
                int incorrect_matches_count = 0;
                for (int objA = 0; objA < objects_count; ++objA) {
                    // все сопоставления исходящие из сторон этого объекта - будут одного случайного цвета
                    color8u random_color_for_object = {(uint8_t) r.nextInt(0, 255), (uint8_t) r.nextInt(0, 255), (uint8_t) r.nextInt(0, 255)};
                    point2i random_shift = {r.nextInt(-segment_thickness, segment_thickness), r.nextInt(-segment_thickness, segment_thickness)}; // это нужно чтобы встречные ребра не наслоились закрыв друг друга, а было легко видеть что это два ребра
                    for (int sideA = 0; sideA < objSides[objA].size(); ++sideA) {
                        auto [objB, sideB, differenceBest, differenceSecondBest] = objMatchedSides[objA][sideA];

                        if (correct_matches.count(image_name)) {
                            auto [expectedObjB, expectedSideB, _, __] = correct_matches[image_name][objA][sideA];
                            if (expectedObjB == objB && expectedSideB == sideB) {
                                correct_matches_count++;
                            } else {
                                incorrect_matches_count++;
                                std::cerr << "EXPECTED: obj" << objA << "-side" <<sideA << " -> obj" << expectedObjB << "-side" << expectedSideB << " with difference=" << differenceBest << " (second best: " << differenceSecondBest << ")" << " - BUT FOUND:" << std::endl;
                                if (differenceBest == -1) {
                                    std::cerr << "obj" << objA << "-side" <<sideA << " -> obj" << objB << "-side" << sideB << " with difference=" << differenceBest << " (second best: " << differenceSecondBest << ")" << std::endl;
                                }
                            }
                        }

                        if (differenceBest == -1) {
                            continue;
                        }

                        std::cout << "obj" << objA << "-side" <<sideA << " -> obj" << objB << "-side" << sideB << " with difference=" << differenceBest << " (second best: " << differenceSecondBest << ")" << std::endl;
                        point2i sideACenter = objOffsets[objA] + objSides[objA][sideA][objSides[objA][sideA].size() / 2]; // вершина в середине стороны A
                        point2i sideBCenter = objOffsets[objB] + objSides[objB][sideB][objSides[objB][sideB].size() / 2]; // вершина в середине сопоставленной с ней стороны B
                        drawPoint(segments_between_matched_sides, random_shift + sideACenter, random_color_for_object, 4 * segment_thickness);
                        drawSegment(segments_between_matched_sides, random_shift + sideACenter, random_shift + sideBCenter, random_color_for_object, segment_thickness);
                    }
                }
                if (correct_matches.count(image_name)) {
                    std::cout << "correct matches: " << correct_matches_count << std::endl;
                    std::cout << "incorrect matches: " << incorrect_matches_count << std::endl;
                }
                debug_io::dump_image(debug_dir + "08_matched_sides.jpg", segments_between_matched_sides);
            }

            // Занятие 7
            // Итак у нас есть:
            // 1) objOffsets, objImages, objMasks - извлеченные изображения объектов-кусочков (с маской и смещением указывающим на позицию в целой картинке)
            // 2) objSides[obj][side] - vector<point2i> - координаты пикселей стороны side объекта obj (в его извлеченном изображении)
            // 3) objMatchedSides[objA][sideA] = {objB, sideB, ...}; - информация о том с каким (objB, sideB) нас сопоставило, или (-1, -1) если мы являемся белым краем

            // План:
            // 1) Построить граф: вершины - объекты, ребра - сопоставления с другими объектами (из objMatchedSides)
            // 2) Найти в графе вершины-углы - попробовать выложить паззл начиная с них
            // 3) Найдя угол выясним сколько кусочков в ширину - прошагаем по графу вправо до упора
            // 4) Так же выясним сколько кусочков в высоту - шагаем от угла вниз до упора
            // 5) Сверяем что ширина * высоту = числу кусочков (иначе - пропускаем этот уголок, попробуем начать с другого)
            // 6) Создаем двумерный массив, каждая ячейка будет хранить номер кусочка-объекта + число поворотов по часовой стрелке (такое чтобы side0 смотрело направо, соответственно side1 - вниз, и т.д.) - TODO
            // 7) Выводим его для проверки в консоль
            // 8) Заполняем его распространяясь в ширину от кусочка-уголка
            // 9) TODO Определим ширину/высоту каждого столбика/строки пазла (медиана от ширин/высот назначенных кусочков)
            //    пока что в коде сделано наивно - везде ширина и толщина берется за 200 пикселей
            // 10) Найдем для каждого кусочка матрицу описывающую переход из его изображения в общий холст
            // 11) Спроецируем все кусочки этой матрицей
            PuzzleAssemblyResult assembled = assemblePuzzle(objImages, objMasks, objCorners, objMatchedSides);

            printGrid(std::cout, assembled);

            debug_io::dump_image(debug_dir + "09_assembled_with_lines.png", assembled.assembledWithLines);
            debug_io::dump_image(debug_dir + "10_assembled.png", assembled.assembled);

            std::cout << "image " << image_name << " processed in " << total_t.elapsed() << " sec" << std::endl;
        }
        std::cout << "all images processed in " << all_images_t.elapsed() << " sec" << std::endl;

        return 0;
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 2;
    }
}
