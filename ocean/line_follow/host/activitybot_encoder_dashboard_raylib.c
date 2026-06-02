#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include "raylib.h"

#ifndef SIM2REAL_ROOT
#define SIM2REAL_ROOT "."
#endif

#ifndef LINE_FOLLOW_ROOT
#define LINE_FOLLOW_ROOT SIM2REAL_ROOT
#endif

#define TRACE_CAP 4096
#define LOG_CAP 8
#define LOG_LEN 192
#define LINE_CAP 2048

typedef struct {
    double t;
    int left;
    int right;
    int left_raw;
    int right_raw;
} Sample;

typedef struct {
    float values[TRACE_CAP];
    int next;
    int count;
} Trace;

typedef struct {
    char lines[LOG_CAP][LOG_LEN];
    int next;
    int count;
} LogRing;

typedef struct {
    pid_t pid;
    int fd;
    int exit_code;
    bool exited;
    bool stopped_by_parent;
    char pending[LINE_CAP];
    int pending_len;
} LoaderProc;

typedef struct {
    const char *port;
    const char *board;
    const char *loader;
    const char *elf;
    const char *summary_file;
    int width;
    int height;
    double seconds;
} Args;

static const Color C_BG = {15, 18, 24, 255};
static const Color C_PANEL = {27, 31, 39, 255};
static const Color C_PANEL_DARK = {22, 25, 31, 255};
static const Color C_BAR_BG = {36, 39, 46, 255};
static const Color C_LINE_DIM = {80, 86, 96, 255};
static const Color C_GRID = {45, 50, 58, 255};
static const Color C_MUTED = {155, 164, 178, 255};
static const Color C_CYAN = {64, 196, 255, 255};
static const Color C_ORANGE = {255, 176, 64, 255};
static const Color C_GREEN = {94, 220, 126, 255};
static const Color C_OFF = {70, 75, 84, 255};

static double monotonic_seconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1000000000.0;
}

static void sleep_ms(long ms) {
    struct timespec req;
    req.tv_sec = ms / 1000;
    req.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&req, NULL);
}

static const char *env_default(const char *name, const char *fallback) {
    const char *value = getenv(name);
    return (value && value[0]) ? value : fallback;
}

static void path_join(char *out, size_t out_size, const char *root, const char *suffix) {
    snprintf(out, out_size, "%s/%s", root, suffix);
}

static void trace_push(Trace *trace, float value) {
    trace->values[trace->next] = value;
    trace->next = (trace->next + 1) % TRACE_CAP;
    if (trace->count < TRACE_CAP) {
        trace->count++;
    }
}

static float trace_recent(const Trace *trace) {
    if (trace->count == 0) {
        return 0.0f;
    }
    int idx = trace->next - 1;
    if (idx < 0) {
        idx += TRACE_CAP;
    }
    return trace->values[idx];
}

static float trace_at_recent(const Trace *trace, int offset_from_oldest_recent, int recent_count) {
    int oldest = trace->next - trace->count;
    while (oldest < 0) {
        oldest += TRACE_CAP;
    }
    int skip = trace->count - recent_count;
    int idx = (oldest + skip + offset_from_oldest_recent) % TRACE_CAP;
    return trace->values[idx];
}

static void log_push(LogRing *logs, const char *line) {
    if (!line || !line[0]) {
        return;
    }
    snprintf(logs->lines[logs->next], LOG_LEN, "%s", line);
    logs->next = (logs->next + 1) % LOG_CAP;
    if (logs->count < LOG_CAP) {
        logs->count++;
    }
}

static const char *log_at(const LogRing *logs, int offset) {
    int oldest = logs->next - logs->count;
    while (oldest < 0) {
        oldest += LOG_CAP;
    }
    return logs->lines[(oldest + offset) % LOG_CAP];
}

static void draw_label(int x, int y, const char *text, int size, Color color) {
    DrawText(text, x, y, size, color);
}

static void draw_bar(int x, int y, int w, int h, float value, float peak, Color color) {
    DrawRectangle(x, y, w, h, C_BAR_BG);
    int fill = 0;
    if (peak > 0.0f) {
        fill = (int)fminf((float)w, fmaxf(0.0f, (value / peak) * (float)w));
    }
    DrawRectangle(x, y, fill, h, color);
    DrawRectangleLines(x, y, w, h, C_MUTED);
}

static void draw_trace(int x, int y, int w, int h, const Trace *trace, float peak, Color color) {
    DrawRectangle(x, y, w, h, C_PANEL_DARK);
    DrawRectangleLines(x, y, w, h, C_LINE_DIM);
    DrawLine(x, y + h / 2, x + w, y + h / 2, C_GRID);

    int recent = trace->count < w ? trace->count : w;
    if (recent < 2) {
        return;
    }

    float scale = fmaxf(peak, 1.0f);
    int start_x = x + (w - recent);
    float first = trace_at_recent(trace, 0, recent);
    int prev_x = start_x;
    int prev_y = y + h - (int)((first / scale) * (float)(h - 8)) - 4;
    if (prev_y < y + 3) prev_y = y + 3;
    if (prev_y > y + h - 3) prev_y = y + h - 3;

    for (int i = 1; i < recent; i++) {
        float value = trace_at_recent(trace, i, recent);
        int px = start_x + i;
        int py = y + h - (int)((value / scale) * (float)(h - 8)) - 4;
        if (py < y + 3) py = y + 3;
        if (py > y + h - 3) py = y + h - 3;
        DrawLine(prev_x, prev_y, px, py, color);
        prev_x = px;
        prev_y = py;
    }
}

static void strip_line(char *line) {
    char *dst = line;
    for (char *src = line; *src; src++) {
        if (*src != '\r') {
            *dst++ = *src;
        }
    }
    *dst = '\0';
    while (*line == ' ' || *line == '\t') {
        memmove(line, line + 1, strlen(line));
    }
    size_t n = strlen(line);
    while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == ' ' || line[n - 1] == '\t')) {
        line[--n] = '\0';
    }
}

static bool parse_sample(const char *line, Sample *sample, double t) {
    int left = 0;
    int right = 0;
    int left_raw = 0;
    int right_raw = 0;

    const char *candidate = line;
    while (candidate && *candidate) {
        if (candidate[0] == 'E' && candidate[1] == ' ') {
            if (sscanf(candidate, "E %d %d %d %d", &left, &right, &left_raw, &right_raw) == 4) {
                sample->t = t;
                sample->left = left;
                sample->right = right;
                sample->left_raw = left_raw;
                sample->right_raw = right_raw;
                return true;
            }
        }
        candidate = strchr(candidate + 1, 'E');
    }
    return false;
}

static bool file_exists(const char *path) {
    return access(path, R_OK) == 0;
}

static int start_loader(LoaderProc *proc, const Args *args, LogRing *logs) {
    int pipefd[2];
    if (pipe(pipefd) != 0) {
        log_push(logs, strerror(errno));
        return -1;
    }

    pid_t pid = fork();
    if (pid < 0) {
        log_push(logs, strerror(errno));
        close(pipefd[0]);
        close(pipefd[1]);
        return -1;
    }

    if (pid == 0) {
        setpgid(0, 0);
        close(pipefd[0]);
        dup2(pipefd[1], STDOUT_FILENO);
        dup2(pipefd[1], STDERR_FILENO);
        close(pipefd[1]);
        execl(args->loader, "propeller-load", "-b", args->board, "-p", args->port, "-r", "-t", args->elf, (char *)NULL);
        perror("exec propeller-load");
        _exit(127);
    }

    close(pipefd[1]);
    int flags = fcntl(pipefd[0], F_GETFL, 0);
    if (flags >= 0) {
        fcntl(pipefd[0], F_SETFL, flags | O_NONBLOCK);
    }

    proc->pid = pid;
    proc->fd = pipefd[0];
    proc->exit_code = -1;
    proc->exited = false;
    proc->stopped_by_parent = false;
    proc->pending_len = 0;
    proc->pending[0] = '\0';
    return 0;
}

static void handle_line(
    const char *raw_line,
    Sample *latest,
    Trace *left_speed,
    Trace *right_speed,
    float *peak_speed,
    int *peak_count,
    LogRing *logs
) {
    char line[LINE_CAP];
    snprintf(line, sizeof(line), "%s", raw_line);
    strip_line(line);
    if (!line[0]) {
        return;
    }

    Sample sample;
    if (parse_sample(line, &sample, monotonic_seconds())) {
        double dt = sample.t - latest->t;
        if (dt < 0.001) {
            dt = 0.001;
        }
        int dl = sample.left - latest->left;
        int dr = sample.right - latest->right;
        if (dl < 0) dl = 0;
        if (dr < 0) dr = 0;

        float ls = (float)((double)dl / dt);
        float rs = (float)((double)dr / dt);
        *peak_speed = fmaxf(fmaxf(*peak_speed * 0.997f, ls), fmaxf(rs, 1.0f));
        if (sample.left > *peak_count) *peak_count = sample.left;
        if (sample.right > *peak_count) *peak_count = sample.right;
        if (*peak_count < 1) *peak_count = 1;
        trace_push(left_speed, ls);
        trace_push(right_speed, rs);
        *latest = sample;
    } else {
        log_push(logs, line);
    }
}

static void pump_loader(
    LoaderProc *proc,
    Sample *latest,
    Trace *left_speed,
    Trace *right_speed,
    float *peak_speed,
    int *peak_count,
    LogRing *logs
) {
    if (proc->fd >= 0) {
        for (;;) {
            char buf[512];
            ssize_t n = read(proc->fd, buf, sizeof(buf));
            if (n > 0) {
                for (ssize_t i = 0; i < n; i++) {
                    char ch = buf[i];
                    if (proc->pending_len < LINE_CAP - 1) {
                        proc->pending[proc->pending_len++] = ch;
                        proc->pending[proc->pending_len] = '\0';
                    }
                    if (ch == '\n') {
                        handle_line(proc->pending, latest, left_speed, right_speed, peak_speed, peak_count, logs);
                        proc->pending_len = 0;
                        proc->pending[0] = '\0';
                    }
                }
                continue;
            }
            if (n == 0) {
                if (proc->pending_len > 0) {
                    handle_line(proc->pending, latest, left_speed, right_speed, peak_speed, peak_count, logs);
                    proc->pending_len = 0;
                    proc->pending[0] = '\0';
                }
                close(proc->fd);
                proc->fd = -1;
                break;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;
            }
            log_push(logs, strerror(errno));
            close(proc->fd);
            proc->fd = -1;
            break;
        }
    }

    if (!proc->exited && proc->pid > 0) {
        int status = 0;
        pid_t result = waitpid(proc->pid, &status, WNOHANG);
        if (result == proc->pid) {
            proc->exited = true;
            if (WIFEXITED(status)) {
                proc->exit_code = WEXITSTATUS(status);
            } else if (WIFSIGNALED(status)) {
                proc->exit_code = 128 + WTERMSIG(status);
            } else {
                proc->exit_code = status;
            }
        }
    }
}

static void stop_loader(LoaderProc *proc) {
    if (proc->pid <= 0 || proc->exited) {
        if (proc->fd >= 0) {
            close(proc->fd);
            proc->fd = -1;
        }
        return;
    }

    kill(-proc->pid, SIGINT);
    proc->stopped_by_parent = true;
    for (int i = 0; i < 20; i++) {
        int status = 0;
        pid_t result = waitpid(proc->pid, &status, WNOHANG);
        if (result == proc->pid) {
            proc->exited = true;
            if (WIFEXITED(status)) {
                proc->exit_code = WEXITSTATUS(status);
            } else if (WIFSIGNALED(status)) {
                proc->exit_code = 128 + WTERMSIG(status);
            }
            break;
        }
        sleep_ms(100);
    }

    if (!proc->exited) {
        kill(-proc->pid, SIGKILL);
        waitpid(proc->pid, NULL, 0);
        proc->exited = true;
        proc->exit_code = 137;
    }

    if (proc->fd >= 0) {
        close(proc->fd);
        proc->fd = -1;
    }
}

static void write_summary(const Args *args, const Sample *latest, float peak_speed, const LogRing *logs) {
    FILE *fp = fopen(args->summary_file, "w");
    if (!fp) {
        return;
    }
    fprintf(fp, "port=%s\n", args->port);
    fprintf(fp, "board=%s\n", args->board);
    fprintf(fp, "left_edges=%d\n", latest->left);
    fprintf(fp, "right_edges=%d\n", latest->right);
    fprintf(fp, "left_raw=%d\n", latest->left_raw);
    fprintf(fp, "right_raw=%d\n", latest->right_raw);
    fprintf(fp, "peak_edges_per_second=%.2f\n", peak_speed);
    fprintf(fp, "logs:\n");
    for (int i = 0; i < logs->count; i++) {
        fprintf(fp, "%s\n", log_at(logs, i));
    }
    fclose(fp);
}

static void print_usage(const char *argv0) {
    fprintf(stderr,
        "Usage: %s [--port /dev/ttyUSB0] [--board activityboard] [--seconds N]\n"
        "          [--width W] [--height H] [--loader PATH] [--elf PATH]\n"
        "          [--summary-file PATH]\n",
        argv0);
}

static int parse_args(int argc, char **argv, Args *args) {
    static char loader_path[1024];
    static char elf_path[1024];
    static char summary_path[1024];

    path_join(loader_path, sizeof(loader_path), SIM2REAL_ROOT, "tools/parallax/simpleide/opt/parallax/bin/propeller-load");
    path_join(elf_path, sizeof(elf_path), LINE_FOLLOW_ROOT, "build/parallax-smoke/encoder_stream.elf");
    path_join(summary_path, sizeof(summary_path), LINE_FOLLOW_ROOT, "build/parallax-smoke/encoder-dashboard-raylib-c-last.txt");

    args->port = env_default("PROPELLER_LOAD_PORT", "/dev/ttyUSB0");
    args->board = env_default("PROPELLER_LOAD_BOARD", "activityboard");
    args->loader = loader_path;
    args->elf = elf_path;
    args->summary_file = summary_path;
    args->width = 1120;
    args->height = 720;
    args->seconds = 0.0;

    for (int i = 1; i < argc; i++) {
        const char *arg = argv[i];
        if (strcmp(arg, "--port") == 0 && i + 1 < argc) {
            args->port = argv[++i];
        } else if (strcmp(arg, "--board") == 0 && i + 1 < argc) {
            args->board = argv[++i];
        } else if (strcmp(arg, "--loader") == 0 && i + 1 < argc) {
            args->loader = argv[++i];
        } else if (strcmp(arg, "--elf") == 0 && i + 1 < argc) {
            args->elf = argv[++i];
        } else if (strcmp(arg, "--summary-file") == 0 && i + 1 < argc) {
            args->summary_file = argv[++i];
        } else if (strcmp(arg, "--width") == 0 && i + 1 < argc) {
            args->width = atoi(argv[++i]);
        } else if (strcmp(arg, "--height") == 0 && i + 1 < argc) {
            args->height = atoi(argv[++i]);
        } else if (strcmp(arg, "--seconds") == 0 && i + 1 < argc) {
            args->seconds = atof(argv[++i]);
        } else if (strcmp(arg, "--help") == 0 || strcmp(arg, "-h") == 0) {
            print_usage(argv[0]);
            return 1;
        } else {
            fprintf(stderr, "Unknown or incomplete argument: %s\n", arg);
            print_usage(argv[0]);
            return -1;
        }
    }

    if (args->width < 720) args->width = 720;
    if (args->height < 520) args->height = 520;
    return 0;
}

int main(int argc, char **argv) {
    Args args;
    int parsed = parse_args(argc, argv, &args);
    if (parsed != 0) {
        return parsed < 0 ? 2 : 0;
    }

    LogRing logs = {0};
    if (!file_exists(args.loader)) {
        fprintf(stderr, "propeller-load not found: %s\n", args.loader);
        return 2;
    }
    if (!file_exists(args.elf)) {
        fprintf(stderr, "encoder stream ELF not found: %s\n", args.elf);
        fprintf(stderr, "Run ocean/line_follow/scripts/parallax-build-encoder-stream.sh first.\n");
        return 2;
    }

    char cmd_log[LOG_LEN];
    snprintf(cmd_log, sizeof(cmd_log), "propeller-load -b %s -p %s -r -t encoder_stream.elf", args.board, args.port);
    log_push(&logs, cmd_log);

    LoaderProc proc = {.pid = -1, .fd = -1, .exit_code = -1, .exited = false, .stopped_by_parent = false, .pending_len = 0};
    if (start_loader(&proc, &args, &logs) != 0) {
        return 2;
    }

    SetTraceLogLevel(LOG_WARNING);
    InitWindow(args.width, args.height, "ActivityBot Encoder Dashboard C");
    SetTargetFPS(60);

    Trace left_speed = {0};
    Trace right_speed = {0};
    Sample latest = {.t = monotonic_seconds(), .left = 0, .right = 0, .left_raw = 0, .right_raw = 0};
    float peak_speed = 1.0f;
    int peak_count = 1;
    double start = monotonic_seconds();

    while (!WindowShouldClose()) {
        pump_loader(&proc, &latest, &left_speed, &right_speed, &peak_speed, &peak_count, &logs);
        if (IsKeyPressed(KEY_Q)) {
            break;
        }
        if (args.seconds > 0.0 && monotonic_seconds() - start >= args.seconds) {
            break;
        }

        int width = GetScreenWidth();
        int height = GetScreenHeight();
        int graph_w = width - 120;
        if (graph_w < 400) graph_w = 400;

        BeginDrawing();
        ClearBackground(C_BG);

        DrawRectangle(28, 24, width - 56, 96, C_PANEL);
        draw_label(48, 42, "ActivityBot Encoder Dashboard", 32, RAYWHITE);

        char line[256];
        snprintf(line, sizeof(line), "%s  |  RAM-only  |  spin wheels by hand  |  Q/Esc closes", args.port);
        draw_label(50, 82, line, 20, C_MUTED);

        snprintf(line, sizeof(line), "LEFT  edges %6d   raw P14=%d   speed %6.1f/s", latest.left, latest.left_raw, trace_recent(&left_speed));
        draw_label(48, 148, line, 24, C_CYAN);
        draw_bar(48, 184, graph_w, 34, (float)latest.left, (float)peak_count, C_CYAN);

        snprintf(line, sizeof(line), "RIGHT edges %6d   raw P15=%d   speed %6.1f/s", latest.right, latest.right_raw, trace_recent(&right_speed));
        draw_label(48, 250, line, 24, C_ORANGE);
        draw_bar(48, 286, graph_w, 34, (float)latest.right, (float)peak_count, C_ORANGE);

        snprintf(line, sizeof(line), "Scrolling speed trace   peak scale %.1f edges/s", peak_speed);
        draw_label(48, 356, line, 22, RAYWHITE);
        draw_trace(48, 390, graph_w, 86, &left_speed, peak_speed, C_CYAN);
        draw_trace(48, 500, graph_w, 86, &right_speed, peak_speed, C_ORANGE);
        draw_label(60, 396, "LEFT", 18, C_CYAN);
        draw_label(60, 506, "RIGHT", 18, C_ORANGE);

        DrawCircle(width - 54, 160, 11, latest.left_raw ? C_GREEN : C_OFF);
        DrawCircle(width - 54, 262, 11, latest.right_raw ? C_GREEN : C_OFF);

        int log_y = height - 140;
        draw_label(48, log_y, "loader / serial", 18, C_MUTED);
        int visible_logs = logs.count < 5 ? logs.count : 5;
        int first_log = logs.count - visible_logs;
        for (int i = 0; i < visible_logs; i++) {
            draw_label(48, log_y + 24 + i * 20, log_at(&logs, first_log + i), 18, C_MUTED);
        }

        if (proc.exited) {
            snprintf(line, sizeof(line), "propeller-load exited: %d", proc.exit_code);
            draw_label(48, height - 36, line, 18, RED);
        } else {
            draw_label(48, height - 36, "live from P14/P15 through usbipd -> /dev/ttyUSB0 -> propeller-load terminal mode", 18, C_MUTED);
        }

        EndDrawing();
    }

    stop_loader(&proc);
    CloseWindow();
    write_summary(&args, &latest, peak_speed, &logs);
    if (proc.stopped_by_parent) {
        return 0;
    }
    return (proc.exit_code == -1 || proc.exit_code == 0 || proc.exit_code == 130) ? 0 : proc.exit_code;
}
