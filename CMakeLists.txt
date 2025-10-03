cmake_minimum_required(VERSION 3.10)

project(first_lab)

# Включение генерации compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Указываем стандарт C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Собираем все исходные файлы
file(GLOB SRC "*.cpp")

# Создаем исполняемый файл
add_executable(first_lab ${SRC})

# Включаем директорию проекта (можете добавить конкретные директории, если нужно)
target_include_directories(first_lab PRIVATE ${CMAKE_SOURCE_DIR})

# Общие компиляторские опции
target_compile_options(first_lab PRIVATE -Wall)

# Конфигурации для Debug
set(DEBUG_COMPILE_OPTIONS -O0 -fsanitize=thread)
target_compile_options(first_lab PRIVATE $<$<CONFIG:Debug>:${DEBUG_COMPILE_OPTIONS}>)

# Конфигурации для Release
set(RELEASE_COMPILE_OPTIONS -O3)
target_compile_options(first_lab PRIVATE $<$<CONFIG:Release>:${RELEASE_COMPILE_OPTIONS}>)
