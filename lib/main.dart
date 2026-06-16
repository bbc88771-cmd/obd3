// OBD Scanner — единый файл (BLE + Wi-Fi).
// Всё приложение в одном файле, чтобы исключить путаницу между файлами.
import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:math';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:path_provider/path_provider.dart';
import 'package:share_plus/share_plus.dart';



// ===== из obd_transport.dart =====

/// Состояние соединения. Используется конечным автоматом в Riverpod.
enum LinkState { disconnected, scanning, connecting, initializing, connected, error }

/// Единый контракт для всех физических каналов связи с ELM327.
///
/// Благодаря этой абстракции доменный слой (ObdService) НЕ знает,
/// через что он говорит с адаптером — BLE, Bluetooth Classic или Wi-Fi.
/// Чтобы добавить новый транспорт — достаточно реализовать этот интерфейс.
abstract class ObdTransport {
  /// Установить физическое соединение (скан + подключение).
  Future<void> connect();

  /// Разорвать соединение и освободить ресурсы.
  Future<void> disconnect();

  /// Отправить ASCII-команду адаптеру.
  /// Завершающий символ \r (0x0D) добавляет сама реализация.
  Future<void> write(String command);

  /// Сырой поток байтов ОТ адаптера. Приходит кусками (chunks),
  /// сборкой целого ответа занимается слой выше (ObdConnection).
  Stream<List<int>> get incoming;

  bool get isConnected;

  /// Поток изменений состояния линка (для авто-reconnect и UI).
  Stream<LinkState> get linkState;
}


// ===== из obd_parsers.dart =====

/// Чистые функции разбора HEX-ответов ЭБУ. Без состояния — легко тестировать.
///
/// Общая логика OBD-II:
///   запрос  = <Mode><PID>           напр. "010C"  (Mode 01, PID 0C)
///   ответ   = <Mode+0x40><PID><данные A B C ...>
///             напр. "410C1AF8" → Mode 41 (=01+0x40), PID 0C, A=1A, B=F8
class ObdParsers {
  /// Маркеры мусорных/служебных ответов адаптера.
  static bool _isGarbage(String r) {
    return r.contains("NODATA") ||
        r.contains("ERROR") ||
        r.contains("SEARCHING") ||
        r.contains("UNABLE") ||
        r.contains("STOPPED") ||
        r.contains("BUFFERFULL") ||
        r.contains("CANERROR") ||
        r.isEmpty;
  }

  /// Достаёт байты данных (A, B, ...) из ответа, проверяя эхо Mode+PID.
  /// Возвращает null, если ответ битый или не тот PID.
  static List<int>? extractData(String resp, String mode, String pid) {
    if (_isGarbage(resp)) return null;

    // режем строку на пары символов = байты
    final bytes = <int>[];
    for (int i = 0; i + 2 <= resp.length; i += 2) {
      final b = int.tryParse(resp.substring(i, i + 2), radix: 16);
      if (b == null) return null; // мусорный символ
      bytes.add(b);
    }

    final modeByte = int.parse(mode, radix: 16) + 0x40; // 01 -> 41
    final pidByte = int.parse(pid, radix: 16);

    // ищем начало валидного фрейма (может быть мусор в начале строки)
    for (int i = 0; i + 1 < bytes.length; i++) {
      if (bytes[i] == modeByte && bytes[i + 1] == pidByte) {
        return bytes.sublist(i + 2); // только полезная нагрузка
      }
    }
    return null;
  }

  /// RPM (PID 0C): (A*256 + B) / 4.
  /// Два байта, разрешение ¼ об/мин → диапазон 0..16383.75.
  /// Пример: A=0x1A(26), B=0xF8(248) → (26*256+248)/4 = 6904/4 = 1726 RPM.
  static int? rpm(String resp) {
    final d = extractData(resp, "01", "0C");
    if (d == null || d.length < 2) return null;
    final v = ((d[0] * 256) + d[1]) ~/ 4;
    return (v >= 0 && v <= 16383) ? v : null; // sanity-check
  }

  /// Скорость (PID 0D): A км/ч напрямую. Один байт 0..255.
  static int? speed(String resp) {
    final d = extractData(resp, "01", "0D");
    if (d == null || d.isEmpty) return null;
    final v = d[0];
    return (v <= 255) ? v : null;
  }

  /// Температура ОЖ (PID 05): A - 40 (°C).
  /// Смещение −40 позволяет кодировать минус без знакового байта
  /// (байт 0x00 = −40 °C, 0xFF = 215 °C).
  static int? coolant(String resp) {
    final d = extractData(resp, "01", "05");
    if (d == null || d.isEmpty) return null;
    return d[0] - 40;
  }

  /// Нагрузка двигателя (PID 04): A * 100 / 255 (%).
  static double? engineLoad(String resp) {
    final d = extractData(resp, "01", "04");
    if (d == null || d.isEmpty) return null;
    return d[0] * 100 / 255;
  }

  /// Положение дросселя (PID 11): A * 100 / 255 (%).
  static double? throttle(String resp) {
    final d = extractData(resp, "01", "11");
    if (d == null || d.isEmpty) return null;
    return d[0] * 100 / 255;
  }

  /// Напряжение по команде ELM327 "ATRV" (не OBD-PID, особый ответ "12.3V").
  /// Из-за рассинхронизации очереди на дешёвых адаптерах сюда иногда попадает
  /// ответ другого PID (напр. "410419"), поэтому достаём первое число регуляркой
  /// и проверяем правдоподобный диапазон бортовой сети (5..30 В).
  static double? voltage(String resp) {
    final m = RegExp(r'(\d{1,2}(?:\.\d+)?)').firstMatch(resp.replaceAll(',', '.'));
    if (m == null) return null;
    final v = double.tryParse(m.group(1)!);
    if (v == null || v < 5 || v > 30) return null;
    return v;
  }

  // ─────────────── РАСШИРЕННЫЕ PID ───────────────

  /// Температура впускного воздуха (PID 0F): A - 40 (°C). Та же логика смещения.
  static int? intakeTemp(String resp) {
    final d = extractData(resp, "01", "0F");
    if (d == null || d.isEmpty) return null;
    return d[0] - 40;
  }

  /// Уровень топлива в баке (PID 2F): A * 100 / 255 (%).
  static double? fuelLevel(String resp) {
    final d = extractData(resp, "01", "2F");
    if (d == null || d.isEmpty) return null;
    return d[0] * 100 / 255;
  }

  /// Расход воздуха MAF (PID 10): (A*256 + B) / 100 (грамм/сек).
  /// Два байта, разрешение 0.01 г/с. Нужен для расчёта мгновенного расхода топлива.
  static double? maf(String resp) {
    final d = extractData(resp, "01", "10");
    if (d == null || d.length < 2) return null;
    return ((d[0] * 256) + d[1]) / 100;
  }

  /// Давление во впускном коллекторе MAP (PID 0B): A кПа напрямую (абсолютное).
  /// Для турбо: наддув = MAP - атмосферное(~101 кПа).
  static int? mapPressure(String resp) {
    final d = extractData(resp, "01", "0B");
    if (d == null || d.isEmpty) return null;
    return d[0];
  }

  /// Давление топлива (PID 0A): A * 3 (кПа). Множитель 3 = разрешение датчика.
  static int? fuelPressure(String resp) {
    final d = extractData(resp, "01", "0A");
    if (d == null || d.isEmpty) return null;
    return d[0] * 3;
  }

  /// Угол опережения зажигания (PID 0E): A/2 - 64 (°).
  /// Смещение −64 позволяет кодировать отрицательные углы (запаздывание).
  static double? timingAdvance(String resp) {
    final d = extractData(resp, "01", "0E");
    if (d == null || d.isEmpty) return null;
    return d[0] / 2 - 64;
  }

  /// Температура окружающего воздуха (PID 46): A - 40 (°C).
  static int? ambientTemp(String resp) {
    final d = extractData(resp, "01", "46");
    if (d == null || d.isEmpty) return null;
    return d[0] - 40;
  }

  /// Обороты по двум байтам — общий помощник, если нужен другой PID с той же формулой.
  /// Лямбда O2 (PID 24, широкополосный): ((A*256+B)/65536)*2 — коэффициент эквивалентности.
  static double? lambda(String resp) {
    final d = extractData(resp, "01", "24");
    if (d == null || d.length < 2) return null;
    return ((d[0] * 256) + d[1]) / 65536 * 2;
  }

  /// Пробег с момента сброса ошибок (PID 31): A*256 + B (км).
  static int? distanceSinceClear(String resp) {
    final d = extractData(resp, "01", "31");
    if (d == null || d.length < 2) return null;
    return (d[0] * 256) + d[1];
  }

  /// Расчётный мгновенный расход топлива (л/ч) из MAF.
  /// Формула для бензина (AFR≈14.7, плотность≈0.745 кг/л):
  ///   л/ч = (MAF[г/с] * 3600) / (14.7 * 745)
  static double? fuelRateFromMaf(double mafGramsPerSec) {
    return (mafGramsPerSec * 3600) / (14.7 * 745);
  }

  /// Чтение DTC (Mode 03). Каждый код = 2 байта.
  ///   старшие 2 бита 1-го байта → буква системы (P/C/B/U)
  ///   биты 5-4 → первая цифра, биты 3-0 → вторая цифра
  ///   2-й байт → последние две шестнадцатеричные цифры
  /// Пример: байты 01 33 → P0133.
  static List<String> dtc(String resp) => dtcFrom(resp, "43");

  /// Универсальный разбор кодов для режимов с эхом echo:
  /// "43" — сохранённые (Mode 03), "47" — ожидающие (Mode 07),
  /// "4A" — постоянные (Mode 0A).
  static List<String> dtcFrom(String resp, String echo) {
    if (_isGarbage(resp)) return [];
    final idx = resp.indexOf(echo);
    if (idx < 0) return [];
    final hex = resp.substring(idx + echo.length);
    final codes = <String>[];
    const letters = ['P', 'C', 'B', 'U'];

    for (int i = 0; i + 4 <= hex.length; i += 4) {
      final chunk = hex.substring(i, i + 4);
      if (chunk == "0000") continue; // пустой слот
      final first = int.tryParse(chunk.substring(0, 2), radix: 16);
      if (first == null) continue;
      final letter = letters[(first & 0xC0) >> 6];   // биты 7-6
      final d1 = (first & 0x30) >> 4;                // биты 5-4
      final d2 = first & 0x0F;                       // биты 3-0
      final rest = chunk.substring(2);               // байт B как есть
      final code = "$letter$d1${d2.toRadixString(16)}$rest".toUpperCase();
      if (!codes.contains(code)) codes.add(code);
    }
    return codes;
  }

  /// VIN (Mode 09, PID 02). Ответ многокадровый ISO-TP, но после нормализации
  /// (ATS0) приходит склеенной строкой. Ищем эхо "4902", пропускаем байт
  /// счётчика сообщений и переводим оставшиеся байты в ASCII.
  /// Убирает маркеры строк многокадрового ISO-TP ответа вида "0:", "1:", ...
  /// ELM327 вставляет их перед каждым кадром; в чистом HEX символа ':' нет.
  static String _stripFrameMarkers(String s) =>
      s.replaceAll(RegExp(r'[0-9A-F]:'), '');

  static String? vin(String resp) {
    resp = _stripFrameMarkers(resp);
    final idx = resp.indexOf("4902");
    if (idx < 0) return null;
    var hex = resp.substring(idx + 4);
    if (hex.length >= 2) hex = hex.substring(2); // байт-счётчик (обычно 01)
    final sb = StringBuffer();
    for (int i = 0; i + 2 <= hex.length; i += 2) {
      final b = int.tryParse(hex.substring(i, i + 2), radix: 16);
      if (b == null) continue;
      if (b >= 0x20 && b < 0x7F) sb.write(String.fromCharCode(b));
    }
    // VIN не содержит I, O, Q — отбрасываем мусор и берём 17 последних символов.
    final clean = sb.toString().replaceAll(RegExp(r'[^A-HJ-NPR-Z0-9]'), '');
    if (clean.length < 11) return null;
    return clean.length > 17 ? clean.substring(clean.length - 17) : clean;
  }

  /// Текстовый ответ Mode 09 (имя ЭБУ 090A, Calibration ID 0904 и т.п.).
  /// echoPrefix — например "490A". Возвращает читаемую ASCII-строку.
  static String? mode09Text(String resp, String echoPrefix) {
    resp = _stripFrameMarkers(resp);
    final idx = resp.indexOf(echoPrefix);
    if (idx < 0) return null;
    var hex = resp.substring(idx + echoPrefix.length);
    if (hex.length >= 2) hex = hex.substring(2); // байт-счётчик
    final sb = StringBuffer();
    for (int i = 0; i + 2 <= hex.length; i += 2) {
      final b = int.tryParse(hex.substring(i, i + 2), radix: 16);
      if (b == null) continue;
      if (b >= 0x20 && b < 0x7F) sb.write(String.fromCharCode(b));
    }
    final s = sb.toString().trim();
    return s.isEmpty ? null : s;
  }

  /// Данные стоп-кадра (Mode 02). Ответ: 42 <PID> <frame#> <данные...>.
  /// Возвращает только полезные байты (без mode/pid/frame).
  static List<int>? freezeData(String resp, String pid) {
    if (_isGarbage(resp)) return null;
    final bytes = <int>[];
    for (int i = 0; i + 2 <= resp.length; i += 2) {
      final b = int.tryParse(resp.substring(i, i + 2), radix: 16);
      if (b == null) return null;
      bytes.add(b);
    }
    final pidByte = int.parse(pid, radix: 16);
    for (int i = 0; i + 3 <= bytes.length; i++) {
      if (bytes[i] == 0x42 && bytes[i + 1] == pidByte) {
        return bytes.sublist(i + 3); // пропускаем mode(42), pid, frame#
      }
    }
    return null;
  }

  /// Готовность мониторов (Mode 01, PID 01). Используется для теста на выбросы.
  /// A: бит7 — лампа Check Engine, биты6-0 — число сохранённых DTC.
  /// B: поддержка/готовность непрерывных мониторов.
  /// C/D: поддержка/готовность некоторых периодических мониторов.
  static Readiness? readiness(String resp) {
    final d = extractData(resp, "01", "01");
    if (d == null || d.length < 4) return null;
    final a = d[0], b = d[1], c = d[2], e = d[3];

    final monitors = <MonitorStatus>[
      // Непрерывные мониторы (байт B): бит поддержки vs бит готовности (+4).
      MonitorStatus("Пропуски зажигания", (b & 0x01) != 0, (b & 0x10) == 0),
      MonitorStatus("Топливная система", (b & 0x02) != 0, (b & 0x20) == 0),
      MonitorStatus("Компоненты двигателя", (b & 0x04) != 0, (b & 0x40) == 0),
    ];
    // Периодические мониторы: C — поддержка, D — готовность (0 = завершён).
    const names = [
      "Катализатор",
      "Подогрев катализатора",
      "Система EVAP",
      "Вторичный воздух",
      "Хладагент A/C",
      "Кислородный датчик",
      "Подогрев кисл. датчика",
      "Система EGR/VVT",
    ];
    for (int i = 0; i < 8; i++) {
      monitors.add(MonitorStatus(
        names[i],
        (c & (1 << i)) != 0,
        (e & (1 << i)) == 0,
      ));
    }
    return Readiness(milOn: (a & 0x80) != 0, dtcCount: a & 0x7F, monitors: monitors);
  }
}

/// Сводка готовности бортовых мониторов для теста на выбросы.
class Readiness {
  final bool milOn;
  final int dtcCount;
  final List<MonitorStatus> monitors;
  const Readiness({
    required this.milOn,
    required this.dtcCount,
    required this.monitors,
  });
}

/// Состояние одного диагностического монитора.
class MonitorStatus {
  final String name;
  final bool supported;
  final bool complete; // true = тест завершён (готов)
  const MonitorStatus(this.name, this.supported, this.complete);
}

/// Расшифрованный код ошибки.
class DtcInfo {
  final String code;
  final String system;      // подсистема (Двигатель/Шасси/Кузов/Сеть)
  final bool generic;       // true = общий стандарт SAE, false = код производителя
  final String description; // человекочитаемое описание
  const DtcInfo({
    required this.code,
    required this.system,
    required this.generic,
    required this.description,
  });
}

/// Небольшой встроенный справочник распространённых кодов + структурный разбор.
class DtcCatalog {
  static const _db = <String, String>{
    "P0100": "Неисправность цепи расходомера воздуха (MAF)",
    "P0101": "MAF: показания вне диапазона",
    "P0102": "MAF: низкий сигнал",
    "P0113": "Датчик температуры впуска (IAT): высокий сигнал",
    "P0117": "Датчик температуры ОЖ: низкий сигнал",
    "P0118": "Датчик температуры ОЖ: высокий сигнал",
    "P0120": "Неисправность цепи датчика положения дросселя (TPS)",
    "P0128": "Термостат: ОЖ не достигает рабочей температуры",
    "P0130": "Датчик кислорода (Bank 1, Sensor 1): цепь",
    "P0131": "Датчик кислорода (B1S1): низкое напряжение",
    "P0133": "Медленный отклик датчика кислорода (B1S1)",
    "P0134": "Нет активности датчика кислорода (B1S1)",
    "P0135": "Подогрев датчика кислорода (B1S1): цепь",
    "P0171": "Слишком бедная смесь (Bank 1)",
    "P0172": "Слишком богатая смесь (Bank 1)",
    "P0174": "Слишком бедная смесь (Bank 2)",
    "P0300": "Случайные/множественные пропуски зажигания",
    "P0301": "Пропуски зажигания в цилиндре 1",
    "P0302": "Пропуски зажигания в цилиндре 2",
    "P0303": "Пропуски зажигания в цилиндре 3",
    "P0304": "Пропуски зажигания в цилиндре 4",
    "P0325": "Датчик детонации: цепь (Bank 1)",
    "P0335": "Датчик положения коленвала (CKP): цепь",
    "P0340": "Датчик положения распредвала (CMP): цепь",
    "P0401": "Недостаточный поток рециркуляции ОГ (EGR)",
    "P0420": "Эффективность катализатора ниже порога (Bank 1)",
    "P0430": "Эффективность катализатора ниже порога (Bank 2)",
    "P0440": "Система улавливания паров топлива (EVAP): утечка",
    "P0442": "EVAP: малая утечка",
    "P0455": "EVAP: большая утечка",
    "P0500": "Датчик скорости автомобиля (VSS): неисправность",
    "P0506": "Обороты холостого хода ниже нормы",
    "P0507": "Обороты холостого хода выше нормы",
    "P0600": "Ошибка шины обмена ЭБУ",
    "P0700": "Неисправность системы управления АКПП",
    "U0100": "Потеря связи с ЭБУ двигателя (ECM/PCM)",
    "U0121": "Потеря связи с блоком ABS",
    "C0035": "Датчик скорости левого переднего колеса",
    "B0010": "Подушка безопасности: цепь",
  };

  static DtcInfo describe(String raw) {
    final code = raw.toUpperCase().trim();
    String system;
    switch (code.isEmpty ? 'P' : code[0]) {
      case 'C': system = "Шасси (ABS, подвеска)"; break;
      case 'B': system = "Кузов (комфорт, SRS)"; break;
      case 'U': system = "Сеть и обмен данными"; break;
      default: system = "Двигатель и трансмиссия";
    }
    // Вторая цифра: 0 — общий стандарт SAE, 1 — код производителя.
    final generic = code.length > 1 && (code[1] == '0' || code[1] == '2');
    final desc = _db[code] ??
        (generic
            ? "Стандартный код $system. Описание не в базе — уточните по справочнику."
            : "Код производителя — описание зависит от марки авто.");
    return DtcInfo(code: code, system: system, generic: generic, description: desc);
  }
}

/// Каталог PID для удобного опроса.
class Pid {
  final String label;
  final String cmd;     // что слать, напр. "010C"
  final String unit;
  const Pid(this.label, this.cmd, this.unit);

  static const rpm = Pid("Обороты", "010C", "об/мин");
  static const speed = Pid("Скорость", "010D", "км/ч");
  static const coolant = Pid("Темп. ОЖ", "0105", "°C");
  static const load = Pid("Нагрузка", "0104", "%");
  static const throttle = Pid("Дроссель", "0111", "%");

  // расширенные
  static const intakeTemp = Pid("Темп. впуска", "010F", "°C");
  static const fuelLevel = Pid("Топливо", "012F", "%");
  static const maf = Pid("MAF", "0110", "г/с");
  static const map = Pid("Наддув (MAP)", "010B", "кПа");
  static const fuelPressure = Pid("Давл. топлива", "010A", "кПа");
  static const timing = Pid("Опереж. зажиг.", "010E", "°");
  static const ambient = Pid("Темп. за бортом", "0146", "°C");
  static const lambda = Pid("Лямбда", "0124", "λ");
  static const distance = Pid("Пробег с DTC", "0131", "км");
}


// ===== из obd_connection.dart =====

/// Превращает «рваный» поток байтов от адаптера в цельные текстовые ответы.
///
/// ELM327 завершает КАЖДЫЙ ответ символом-приглашением '>' (0x3E).
/// Байты могут приходить кусками ("41 0C", потом "1A F8\r>"),
/// поэтому копим буфер до появления '>'.
class ObdConnection {
  final ObdTransport transport;
  final StringBuffer _buffer = StringBuffer();
  Completer<String>? _pending;

  late final StreamSubscription _sub;

  ObdConnection(this.transport) {
    _sub = transport.incoming.listen(_onBytes);
  }

  void _onBytes(List<int> chunk) {
    _buffer.write(String.fromCharCodes(chunk));
    final current = _buffer.toString();

    if (current.contains('>')) {
      _buffer.clear();
      // нормализация: убираем приглашение, переводы строк и пробелы.
      // (ATS0 обычно уже убрал пробелы, но подстраховываемся)
      final clean = current
          .replaceAll('>', '')
          .replaceAll('\r', '')
          .replaceAll('\n', '')
          .replaceAll(' ', '')
          .toUpperCase()
          .trim();

      final p = _pending;
      _pending = null;
      if (p != null && !p.isCompleted) p.complete(clean);
    }
  }

  /// Отправляет одну команду и ждёт цельный ответ или таймаут.
  Future<String> send(
    String cmd, {
    Duration timeout = const Duration(milliseconds: 2500),
  }) {
    _buffer.clear(); // чистим хвосты предыдущего обмена
    final completer = Completer<String>();
    _pending = completer;

    transport.write(cmd);

    return completer.future.timeout(timeout, onTimeout: () {
      _pending = null;
      throw TimeoutException("Таймаут ответа на '$cmd'");
    });
  }

  Future<void> dispose() async => _sub.cancel();
}


// ===== из request_queue.dart =====

/// Последовательная очередь команд.
///
/// ELM327 полудуплексный и обрабатывает РОВНО одну команду за раз.
/// Если послать вторую до прихода '>', ответы слипнутся в кашу.
/// Очередь гарантирует строгий порядок «запрос → ответ → следующий».
class RequestQueue {
  final ObdConnection conn;
  final List<_Item> _items = [];
  bool _pumping = false;

  /// Микропауза между командами — дешёвым клонам нужно «продохнуть».
  final Duration gap;

  RequestQueue(this.conn, {this.gap = const Duration(milliseconds: 25)});

  /// Поставить команду в очередь. Возвращает future с её ответом.
  Future<String> enqueue(String cmd, {Duration? timeout}) {
    final item = _Item(cmd, Completer<String>(), timeout);
    _items.add(item);
    _pump(); // запускаем «насос», если стоял
    return item.completer.future;
  }

  Future<void> _pump() async {
    if (_pumping) return;
    _pumping = true;

    while (_items.isNotEmpty) {
      final item = _items.removeAt(0);
      try {
        final resp = item.timeout != null
            ? await conn.send(item.cmd, timeout: item.timeout!)
            : await conn.send(item.cmd);
        if (!item.completer.isCompleted) item.completer.complete(resp);
      } catch (e) {
        // один таймаут НЕ должен ронять всю очередь
        if (!item.completer.isCompleted) item.completer.completeError(e);
      }
      await Future.delayed(gap);
    }

    _pumping = false;
  }

  void clear() {
    for (final i in _items) {
      if (!i.completer.isCompleted) {
        i.completer.completeError(StateError("Очередь очищена"));
      }
    }
    _items.clear();
  }
}

class _Item {
  final String cmd;
  final Completer<String> completer;
  final Duration? timeout;
  _Item(this.cmd, this.completer, this.timeout);
}


// ===== из obd_service.dart =====

/// Снимок телеметрии для UI.
class Telemetry {
  final int? rpm;
  final int? speed;
  final int? coolant;
  final double? load;
  final double? voltage;
  // расширенные
  final int? intakeTemp;
  final double? fuelLevel;
  final double? maf;
  final int? map;
  final double? timing;
  final int? ambient;
  final double? fuelRate; // л/ч, расчётный из MAF

  const Telemetry({
    this.rpm,
    this.speed,
    this.coolant,
    this.load,
    this.voltage,
    this.intakeTemp,
    this.fuelLevel,
    this.maf,
    this.map,
    this.timing,
    this.ambient,
    this.fuelRate,
  });

  Telemetry copyWith({
    int? rpm,
    int? speed,
    int? coolant,
    double? load,
    double? voltage,
    int? intakeTemp,
    double? fuelLevel,
    double? maf,
    int? map,
    double? timing,
    int? ambient,
    double? fuelRate,
  }) =>
      Telemetry(
        rpm: rpm ?? this.rpm,
        speed: speed ?? this.speed,
        coolant: coolant ?? this.coolant,
        load: load ?? this.load,
        voltage: voltage ?? this.voltage,
        intakeTemp: intakeTemp ?? this.intakeTemp,
        fuelLevel: fuelLevel ?? this.fuelLevel,
        maf: maf ?? this.maf,
        map: map ?? this.map,
        timing: timing ?? this.timing,
        ambient: ambient ?? this.ambient,
        fuelRate: fuelRate ?? this.fuelRate,
      );
}

/// Главный доменный слой: инициализация адаптера, цикл опроса,
/// чтение/сброс ошибок и контроль безопасности.
class ObdService {
  final ObdTransport transport;
  late final ObdConnection _conn;
  late final RequestQueue _queue;

  final _telemetry = StreamController<Telemetry>.broadcast();
  Stream<Telemetry> get telemetry => _telemetry.stream;
  Telemetry _last = const Telemetry();
  Telemetry get lastTelemetry => _last;

  /// Накопленная статистика (min/max/last) по каждому параметру.
  final Map<String, MinMax> _stats = {};
  Map<String, MinMax> get statistics => Map.unmodifiable(_stats);
  void resetStatistics() => _stats.clear();

  Timer? _pollTimer;
  int _consecutiveTimeouts = 0;
  Duration _pollInterval = const Duration(milliseconds: 200);
  bool _pollInFlight = false; // защита от наложения циклов опроса

  ObdService(this.transport) {
    _conn = ObdConnection(transport);
    _queue = RequestQueue(_conn);
  }

  /// Полная инициализация ELM327. Порядок команд важен.
  Future<void> initialize() async {
    // ATZ — полный сброс, адаптер «думает» ~1 сек, даём больше времени.
    await _queue.enqueue("ATZ", timeout: const Duration(seconds: 3));
    await _queue.enqueue("ATE0"); // Echo Off — ответ без копии нашей команды
    await _queue.enqueue("ATL0"); // Linefeed Off — без лишних \n
    await _queue.enqueue("ATS0"); // Spaces Off — компактный HEX "410C1AF8"
    await _queue.enqueue("ATH0"); // Headers Off — на этапе значений не нужны
    await _queue.enqueue("ATSP0"); // авто-определение протокола авто
    // «пробный» OBD-запрос — заставляет адаптер реально выбрать протокол
    await _queue.enqueue("0100", timeout: const Duration(seconds: 5));
    // выясняем, какие PID реально поддерживает ЭБУ этого авто
    await detectSupportedPids();
  }

  /// Запустить периодический опрос (poll loop) с заданной частотой.
  void startPolling({Duration interval = const Duration(milliseconds: 200)}) {
    _pollInterval = interval;
    _pollTimer?.cancel();
    _pollTimer = Timer.periodic(interval, (_) => _pollOnce());
  }

  void stopPolling() {
    _pollTimer?.cancel();
    _pollTimer = null;
  }

  /// Возобновить опрос с ранее заданным интервалом (после паузы).
  void resumePolling() {
    if (_pollTimer == null) startPolling(interval: _pollInterval);
  }

  bool get isPolling => _pollTimer != null;

  /// Сбросить накопившийся хвост команд опроса — чтобы пользовательская
  /// операция (чтение/сброс ошибок) не ждала за длинной очередью телеметрии.
  void clearQueue() => _queue.clear();

  Future<void> _pollOnce() async {
    // На медленном адаптере один цикл может длиться дольше интервала таймера.
    // Без этой защиты вызовы накладывались бы и забивали очередь команд,
    // из-за чего пользовательские запросы (DTC) ждали бы бесконечно.
    if (_pollInFlight) return;
    _pollInFlight = true;
    try {
      await _pollCycle();
    } finally {
      _pollInFlight = false;
    }
  }

  Future<void> _pollCycle() async {
    // Все команды идут через очередь, поэтому не накладываются.
    await _read(Pid.rpm.cmd, (r) {
      final v = ObdParsers.rpm(r);
      if (v != null) _emit(_last.copyWith(rpm: v));
    });
    await _read(Pid.speed.cmd, (r) {
      final v = ObdParsers.speed(r);
      if (v != null) _emit(_last.copyWith(speed: v));
    });
    await _read(Pid.coolant.cmd, (r) {
      final v = ObdParsers.coolant(r);
      if (v != null) _emit(_last.copyWith(coolant: v));
    });
    await _read(Pid.load.cmd, (r) {
      final v = ObdParsers.engineLoad(r);
      if (v != null) _emit(_last.copyWith(load: v));
    });
    await _read("ATRV", (r) {
      final v = ObdParsers.voltage(r);
      if (v != null) _emit(_last.copyWith(voltage: v));
    });

    // расширенные PID опрашиваем только если ЭБУ их поддерживает
    // (проверка через 0100/0120 — список поддержки, см. _detectSupported)
    if (_supports("0F")) {
      await _read(Pid.intakeTemp.cmd, (r) {
        final v = ObdParsers.intakeTemp(r);
        if (v != null) _emit(_last.copyWith(intakeTemp: v));
      });
    }
    if (_supports("2F")) {
      await _read(Pid.fuelLevel.cmd, (r) {
        final v = ObdParsers.fuelLevel(r);
        if (v != null) _emit(_last.copyWith(fuelLevel: v));
      });
    }
    if (_supports("10")) {
      await _read(Pid.maf.cmd, (r) {
        final v = ObdParsers.maf(r);
        if (v != null) {
          // попутно считаем мгновенный расход топлива
          final rate = ObdParsers.fuelRateFromMaf(v);
          _emit(_last.copyWith(maf: v, fuelRate: rate));
        }
      });
    }
    if (_supports("0B")) {
      await _read(Pid.map.cmd, (r) {
        final v = ObdParsers.mapPressure(r);
        if (v != null) _emit(_last.copyWith(map: v));
      });
    }
    if (_supports("0E")) {
      await _read(Pid.timing.cmd, (r) {
        final v = ObdParsers.timingAdvance(r);
        if (v != null) _emit(_last.copyWith(timing: v));
      });
    }
    if (_supports("46")) {
      await _read(Pid.ambient.cmd, (r) {
        final v = ObdParsers.ambientTemp(r);
        if (v != null) _emit(_last.copyWith(ambient: v));
      });
    }
  }

  // ─────────── автоопределение поддерживаемых PID ───────────
  // ЭБУ на запрос 0100 возвращает битовую маску: какие PID 01-20 поддержаны.
  final Set<String> _supported = {};

  // Если детекция поддержки не прошла (частый случай при переподключении),
  // _supported пуст — тогда опрашиваем расширенные PID оптимистично, а не
  // прячем их. Неподдержанные просто вернут NODATA → значение останется пустым.
  bool _supports(String pid) =>
      _supported.isEmpty || _supported.contains(pid.toUpperCase());

  /// Запрашивает маски поддержки (0100, 0120, 0140) и заполняет _supported.
  /// Вызывается один раз после initialize().
  Future<void> detectSupportedPids() async {
    for (final base in ["0100", "0120", "0140"]) {
      try {
        final resp = await _queue.enqueue(base);
        _parseSupportMask(resp, base);
      } catch (_) {/* блок не поддержан — ок */}
    }
  }

  void _parseSupportMask(String resp, String base) {
    // ответ вида 41 00 BE 1F A8 13 → 4 байта маски = 32 бита = PID 01..20
    final pidByte = base.substring(2); // "00","20","40"
    final d = ObdParsers.extractData(resp, "01", pidByte);
    if (d == null || d.length < 4) return;

    final offset = int.parse(pidByte, radix: 16); // 0x00, 0x20, 0x40
    int bit = 0;
    for (final byte in d.take(4)) {
      for (int i = 7; i >= 0; i--) {
        bit++;
        if ((byte & (1 << i)) != 0) {
          final pidNum = offset + bit;
          _supported.add(pidNum.toRadixString(16).padLeft(2, '0').toUpperCase());
        }
      }
    }
  }

  Future<void> _read(String cmd, void Function(String) onOk) async {
    // Опрос мог быть остановлен посреди цикла (пауза на время чтения DTC) —
    // тогда не добавляем оставшиеся команды цикла в очередь.
    if (_pollTimer == null) return;
    try {
      final resp = await _queue.enqueue(cmd);
      _consecutiveTimeouts = 0; // успех — сбрасываем счётчик
      onOk(resp);
    } on TimeoutException {
      // НЕ останавливаем опрос: на дешёвых адаптерах таймауты случаются, но
      // линк жив (обрыв ловит транспорт по linkState). Раньше 3 таймаута
      // насовсем глушили опрос → телеметрия «замерзала» с частью данных.
      _consecutiveTimeouts++;
    } catch (_) {/* битый ответ — просто пропускаем кадр */}
  }

  void _emit(Telemetry t) {
    _last = t;
    _trackStat("Обороты", "об/мин", t.rpm?.toDouble());
    _trackStat("Скорость", "км/ч", t.speed?.toDouble());
    _trackStat("Темп. ОЖ", "°C", t.coolant?.toDouble());
    _trackStat("Нагрузка", "%", t.load);
    _trackStat("Напряжение", "В", t.voltage);
    _trackStat("Темп. впуска", "°C", t.intakeTemp?.toDouble());
    _trackStat("Топливо", "%", t.fuelLevel);
    _trackStat("MAF", "г/с", t.maf);
    _trackStat("Наддув (MAP)", "кПа", t.map?.toDouble());
    _trackStat("Опереж. зажиг.", "°", t.timing);
    _trackStat("Темп. за бортом", "°C", t.ambient?.toDouble());
    _trackStat("Расход", "л/ч", t.fuelRate);
    _telemetry.add(t);
  }

  void _trackStat(String name, String unit, double? value) {
    if (value == null) return;
    final s = _stats[name];
    if (s == null) {
      _stats[name] = MinMax(unit: unit, min: value, max: value, last: value);
    } else {
      s.last = value;
      if (value < s.min) s.min = value;
      if (value > s.max) s.max = value;
    }
  }

  /// Прочитать VIN (Mode 09 PID 02).
  Future<String?> readVin() async {
    final resp = await _queue.enqueue("0902", timeout: const Duration(seconds: 3));
    return ObdParsers.vin(resp);
  }

  /// Прочитать имя/калибровку ЭБУ (Mode 09 PID 0A).
  Future<String?> readEcuName() async {
    final resp = await _queue.enqueue("090A", timeout: const Duration(seconds: 3));
    return ObdParsers.mode09Text(resp, "490A");
  }

  /// Прочитать готовность мониторов выбросов (Mode 01 PID 01).
  Future<Readiness?> readReadiness() async {
    final resp = await _queue.enqueue("0101", timeout: const Duration(seconds: 3));
    return ObdParsers.readiness(resp);
  }

  /// Прочитать стоп-кадр (Mode 02): значения ключевых параметров на момент,
  /// когда ЭБУ зафиксировал ошибку. Запрос — "02 <PID> 00" (кадр 0).
  Future<List<FreezeEntry>> readFreezeFrame() async {
    final result = <FreezeEntry>[];

    Future<void> one(String pid, String label, String unit,
        double? Function(List<int>) calc) async {
      try {
        final resp = await _queue.enqueue("02${pid}00");
        final d = ObdParsers.freezeData(resp, pid);
        if (d == null || d.isEmpty) return;
        final v = calc(d);
        if (v != null) result.add(FreezeEntry(label, v, unit));
      } catch (_) {/* пропускаем недоступный PID */}
    }

    await one("04", "Нагрузка", "%", (d) => d[0] * 100 / 255);
    await one("05", "Темп. ОЖ", "°C", (d) => (d[0] - 40).toDouble());
    await one("0C", "Обороты", "об/мин",
        (d) => d.length < 2 ? null : ((d[0] * 256 + d[1]) / 4));
    await one("0D", "Скорость", "км/ч", (d) => d[0].toDouble());
    await one("11", "Дроссель", "%", (d) => d[0] * 100 / 255);
    await one("10", "MAF", "г/с",
        (d) => d.length < 2 ? null : ((d[0] * 256 + d[1]) / 100));
    return result;
  }

  /// Прочитать сохранённые коды ошибок.
  Future<List<String>> readDtc() async {
    final resp = await _queue.enqueue("03", timeout: const Duration(seconds: 3));
    return ObdParsers.dtc(resp);
  }

  /// Ожидающие коды (Mode 07) — ещё не подтверждённые ЭБУ.
  Future<List<String>> readPendingDtc() async {
    final resp = await _queue.enqueue("07", timeout: const Duration(seconds: 3));
    return ObdParsers.dtcFrom(resp, "47");
  }

  /// Постоянные коды (Mode 0A) — нельзя стереть сканером, гаснут сами.
  Future<List<String>> readPermanentDtc() async {
    final resp = await _queue.enqueue("0A", timeout: const Duration(seconds: 3));
    return ObdParsers.dtcFrom(resp, "4A");
  }

  /// БЕЗОПАСНОСТЬ: проверка, что машина неподвижна перед записью.
  Future<bool> _isSafeToWrite() async {
    try {
      final resp = await _queue.enqueue(Pid.speed.cmd);
      final speed = ObdParsers.speed(resp);
      // нет данных → считаем НЕбезопасным (fail-safe)
      return speed == 0;
    } catch (_) {
      return false;
    }
  }

  /// Сброс DTC (Mode 04). Только на стоящей машине + подтверждение в UI.
  /// Гасит Check Engine и стирает freeze frame — необратимо.
  Future<void> clearDtc({required bool userConfirmed}) async {
    if (!userConfirmed) {
      throw Exception("Требуется подтверждение пользователя");
    }
    if (!await _isSafeToWrite()) {
      throw Exception("Сброс ошибок запрещён: автомобиль должен стоять (V=0)");
    }
    await _queue.enqueue("04", timeout: const Duration(seconds: 3));
  }

  Future<void> dispose() async {
    stopPolling();
    _queue.clear();
    await _conn.dispose();
    await _telemetry.close();
  }
}


// ===== из ble_transport.dart =====

/// Транспорт для ELM327-адаптеров с Bluetooth Low Energy.
///
/// Большинство дешёвых BLE-клонов (Vgate iCar, Veepeak, vLinker) используют
/// сервис FFE0 с одной характеристикой FFE1, которая одновременно
/// принимает запись и шлёт notify. Иногда встречается раздельная пара
/// (write FFE1 + notify FFE2) или сервис 18F0/FFF0 — поэтому ниже идёт
/// перебор кандидатов, а не жёсткая привязка.
class BleTransport implements ObdTransport {
  BluetoothDevice? _device;
  BluetoothCharacteristic? _writeChar;
  BluetoothCharacteristic? _notifyChar;

  StreamSubscription? _notifySub;
  StreamSubscription? _connSub;

  final _incoming = StreamController<List<int>>.broadcast();
  final _state = StreamController<LinkState>.broadcast();

  @override
  Stream<List<int>> get incoming => _incoming.stream;
  @override
  Stream<LinkState> get linkState => _state.stream;
  @override
  bool get isConnected => _device?.isConnected ?? false;

  // Известные UUID сервисов ELM327-BLE клонов (16-битные, развёрнутые в 128).
  static final _candidateServices = <Guid>[
    Guid("0000ffe0-0000-1000-8000-00805f9b34fb"),
    Guid("0000fff0-0000-1000-8000-00805f9b34fb"),
    Guid("000018f0-0000-1000-8000-00805f9b34fb"),
  ];

  /// Сканируем эфир, ищем устройство с «обдшным» именем.
  Future<BluetoothDevice?> _scan() async {
    _state.add(LinkState.scanning);
    BluetoothDevice? found;

    final sub = FlutterBluePlus.scanResults.listen((results) {
      for (final r in results) {
        final name = r.device.platformName.toUpperCase();
        if (name.contains("OBD") ||
            name.contains("VLINK") ||
            name.contains("VEEPEAK") ||
            name.contains("ELM")) {
          found = r.device;
        }
      }
    });

    await FlutterBluePlus.startScan(timeout: const Duration(seconds: 8));
    await FlutterBluePlus.isScanning.where((s) => s == false).first;
    await sub.cancel();
    return found;
  }

  @override
  Future<void> connect() async {
    _device = await _scan();
    if (_device == null) {
      _state.add(LinkState.error);
      throw Exception("BLE OBD-адаптер не найден. Включи зажигание и Bluetooth.");
    }

    _state.add(LinkState.connecting);

    // следим за разрывом для авто-reconnect / UI
    _connSub = _device!.connectionState.listen((s) {
      if (s == BluetoothConnectionState.disconnected) {
        _state.add(LinkState.disconnected);
      }
    });

    await _device!.connect(timeout: const Duration(seconds: 12));

    // некоторым клонам нужен запрос увеличенного MTU,
    // иначе длинные ответы (VIN, multiline DTC) режутся
    try {
      await _device!.requestMtu(180);
    } catch (_) {/* не критично */}

    await _discoverCharacteristics();
    _state.add(LinkState.connected);
  }

  Future<void> _discoverCharacteristics() async {
    final services = await _device!.discoverServices();

    for (final svc in services) {
      if (!_candidateServices.contains(svc.uuid)) continue;

      for (final c in svc.characteristics) {
        final props = c.properties;
        if (props.write || props.writeWithoutResponse) _writeChar = c;
        if (props.notify || props.indicate) _notifyChar = c;
      }
    }

    // фолбэк: если по белому списку не нашли — берём любую подходящую пару
    if (_writeChar == null || _notifyChar == null) {
      for (final svc in services) {
        for (final c in svc.characteristics) {
          if ((c.properties.write || c.properties.writeWithoutResponse) &&
              _writeChar == null) {
            _writeChar = c;
          }
          if ((c.properties.notify || c.properties.indicate) &&
              _notifyChar == null) {
            _notifyChar = c;
          }
        }
      }
    }

    if (_writeChar == null || _notifyChar == null) {
      throw Exception("Не найдены характеристики для обмена данными");
    }

    await _notifyChar!.setNotifyValue(true);
    _notifySub = _notifyChar!.onValueReceived.listen((bytes) {
      _incoming.add(bytes); // отдаём сырые байты наверх
    });
  }

  @override
  Future<void> write(String command) async {
    if (_writeChar == null) throw Exception("Нет канала записи");
    // ВАЖНО: каждая команда ОБЯЗАНА заканчиваться \r, иначе ELM327 её игнорит.
    final data = utf8.encode("$command\r");
    // writeWithoutResponse быстрее; OBD-команды короткие и влезают в MTU.
    await _writeChar!.write(
      data,
      withoutResponse: _writeChar!.properties.writeWithoutResponse,
    );
  }

  @override
  Future<void> disconnect() async {
    await _notifySub?.cancel();
    await _connSub?.cancel();
    await _device?.disconnect();
    _state.add(LinkState.disconnected);
  }
}


// ===== из wifi_transport.dart =====

/// Транспорт для Wi-Fi ELM327-адаптеров.
///
/// Такие адаптеры поднимают собственную точку доступа и работают как
/// TCP-сервер. Классический дефолт у клонов: 192.168.0.10 порт 35000.
/// Телефон должен быть подключён к Wi-Fi-сети самого адаптера.
class WifiTransport implements ObdTransport {
  final String host;
  final int port;

  Socket? _socket;
  StreamSubscription? _sub;

  final _incoming = StreamController<List<int>>.broadcast();
  final _state = StreamController<LinkState>.broadcast();

  WifiTransport({this.host = "192.168.0.10", this.port = 35000});

  @override
  Stream<List<int>> get incoming => _incoming.stream;
  @override
  Stream<LinkState> get linkState => _state.stream;
  @override
  bool get isConnected => _socket != null;

  @override
  Future<void> connect() async {
    _state.add(LinkState.connecting);
    try {
      _socket = await Socket.connect(
        host,
        port,
        timeout: const Duration(seconds: 8),
      );

      // поток байтов из сокета — тот же контракт, что и у BLE
      _sub = _socket!.listen(
        (bytes) => _incoming.add(bytes),
        onError: (_) => _state.add(LinkState.error),
        onDone: () {
          _socket = null;
          _state.add(LinkState.disconnected);
        },
      );

      _state.add(LinkState.connected);
    } catch (e) {
      _state.add(LinkState.error);
      throw Exception("Не удалось подключиться к $host:$port. "
          "Проверь, что телефон в Wi-Fi-сети адаптера.");
    }
  }

  @override
  Future<void> write(String command) async {
    if (_socket == null) throw Exception("Сокет закрыт");
    _socket!.add(utf8.encode("$command\r")); // \r обязателен
    await _socket!.flush();
  }

  @override
  Future<void> disconnect() async {
    await _sub?.cancel();
    await _socket?.close();
    _socket = null;
    _state.add(LinkState.disconnected);
  }
}


// ===== вспомогательные модели =====

/// Накопленный диапазон значения параметра (для экрана статистики).
class MinMax {
  final String unit;
  double min;
  double max;
  double last;
  MinMax({required this.unit, required this.min, required this.max, required this.last});
}

/// Одна строка стоп-кадра.
class FreezeEntry {
  final String label;
  final double value;
  final String unit;
  const FreezeEntry(this.label, this.value, this.unit);
}


// ===== демо-транспорт =====

/// Виртуальный адаптер для режима «Демо»: не требует железа, генерирует
/// правдоподобные ответы ELM327, чтобы можно было посмотреть приложение
/// без подключения к автомобилю. Реализует тот же контракт ObdTransport,
/// поэтому весь доменный слой работает без изменений.
class FakeTransport implements ObdTransport {
  final _incoming = StreamController<List<int>>.broadcast();
  final _state = StreamController<LinkState>.broadcast();
  final _rnd = Random();
  DateTime _start = DateTime.now();
  bool _connected = false;

  @override
  Stream<List<int>> get incoming => _incoming.stream;
  @override
  Stream<LinkState> get linkState => _state.stream;
  @override
  bool get isConnected => _connected;

  @override
  Future<void> connect() async {
    _state.add(LinkState.connecting);
    _start = DateTime.now();
    await Future.delayed(const Duration(milliseconds: 200));
    _connected = true;
    _state.add(LinkState.connected);
  }

  @override
  Future<void> disconnect() async {
    _connected = false;
    _state.add(LinkState.disconnected);
  }

  @override
  Future<void> write(String command) async {
    final resp = _responseFor(command.toUpperCase().trim());
    // ELM327 завершает ответ символом '>'; имитируем задержку линка.
    Future.delayed(Duration(milliseconds: 12 + _rnd.nextInt(20)), () {
      if (!_incoming.isClosed) _incoming.add(utf8.encode("$resp\r\r>"));
    });
  }

  String _hex2(int v) => (v & 0xFF).toRadixString(16).padLeft(2, '0').toUpperCase();
  String _asciiHex(String s) =>
      s.codeUnits.map((c) => _hex2(c)).join();

  String _responseFor(String cmd) {
    final t = DateTime.now().difference(_start).inMilliseconds / 1000.0;

    // Служебные AT-команды.
    if (cmd == "ATZ") return "ELM327 v1.5";
    if (cmd == "ATRV") {
      final v = 13.8 + sin(t / 3) * 0.6;
      return "${v.toStringAsFixed(1)}V";
    }
    if (cmd.startsWith("AT")) return "OK";

    // Маски поддержки PID — заявляем широкую поддержку.
    if (cmd == "0100") return "4100FFFFFFFE";
    if (cmd == "0120") return "4120FFFFFFFF";
    if (cmd == "0140") return "4140FFFFFFFE";

    // Готовность мониторов: лампа выключена, ошибок нет, тесты пройдены.
    if (cmd == "0101") return "410100072100";

    // Коды ошибок (демо): сохранённые P0133, P0420.
    if (cmd == "03") return "4301330420";
    // Ожидающие (Mode 07): P0301.
    if (cmd == "07") return "470301";
    // Постоянные (Mode 0A): P0420.
    if (cmd == "0A") return "4A0420";
    if (cmd == "04") return "44"; // сброс выполнен

    // VIN и имя ЭБУ.
    if (cmd == "0902") return "490201${_asciiHex("WAUZZZ8K9BA123456")}";
    if (cmd == "090A") return "490A01${_asciiHex("ECM-EngineControl")}";

    // Стоп-кадр: 02 <PID> 00 → 42 <PID> 00 <данные>.
    if (cmd.startsWith("02") && cmd.length >= 4) {
      final pid = cmd.substring(2, 4);
      final data = _pidData(pid, t, frozen: true);
      if (data != null) return "42${pid}00$data";
      return "NODATA";
    }

    // Текущие данные Mode 01.
    if (cmd.startsWith("01") && cmd.length >= 4) {
      final pid = cmd.substring(2, 4);
      final data = _pidData(pid, t, frozen: false);
      if (data != null) return "41$pid$data";
      return "NODATA";
    }

    return "NODATA";
  }

  /// HEX-данные для конкретного PID. frozen=true даёт «замороженные» значения.
  String? _pidData(String pid, double t, {required bool frozen}) {
    double wave(double base, double amp, double period) =>
        base + (frozen ? amp * 0.5 : (sin(t / period) * 0.5 + 0.5) * amp);

    switch (pid) {
      case "0C": // RPM
        final rpm = wave(820, 2600, 1.7).round();
        final raw = rpm * 4;
        return _hex2(raw >> 8) + _hex2(raw & 0xFF);
      case "0D": // скорость
        return _hex2(wave(0, 90, 2.3).round());
      case "05": // темп. ОЖ — прогрев к ~90°C
        final temp = (35 + min(55, t * 4)).round();
        return _hex2(temp + 40);
      case "04": // нагрузка
        return _hex2((wave(15, 70, 1.9) * 255 / 100).round());
      case "11": // дроссель
        return _hex2((wave(12, 60, 1.7) * 255 / 100).round());
      case "0F": // темп. впуска
        return _hex2((28 + sin(t / 5) * 4).round() + 40);
      case "2F": // уровень топлива
        return _hex2((62 * 255 / 100).round());
      case "10": // MAF
        final maf = wave(3, 45, 1.8);
        final raw = (maf * 100).round();
        return _hex2(raw >> 8) + _hex2(raw & 0xFF);
      case "0B": // MAP
        return _hex2(wave(30, 120, 1.8).round());
      case "0E": // опережение зажигания
        return _hex2(((wave(8, 22, 2.1) + 64) * 2).round());
      case "46": // темп. за бортом
        return _hex2(19 + 40);
      case "0A": // давление топлива
        return _hex2(128);
      default:
        return null;
    }
  }
}


// ===== из connection_provider.dart =====

/// Доступ к SharedPreferences. Реальный экземпляр подставляется в main()
/// через override, поэтому до инициализации обращаться к нему нельзя.
final prefsProvider = Provider<SharedPreferences>(
  (ref) => throw UnimplementedError("prefsProvider должен быть переопределён в main()"),
);

/// Тип транспорта, выбранный пользователем (BLE или Wi-Fi).
enum TransportKind { ble, wifi }

final transportKindProvider = StateProvider<TransportKind>((ref) {
  final i = ref.watch(prefsProvider).getInt("transportKind");
  return (i != null && i < TransportKind.values.length)
      ? TransportKind.values[i]
      : TransportKind.ble;
});

/// Настройки Wi-Fi-адаптера.
final wifiHostProvider = StateProvider<String>(
    (ref) => ref.watch(prefsProvider).getString("wifiHost") ?? "192.168.0.10");
final wifiPortProvider = StateProvider<int>(
    (ref) => ref.watch(prefsProvider).getInt("wifiPort") ?? 35000);

/// Частота опроса (мс между циклами).
// 500 мс по умолчанию: дешёвым ELM327-клонам 200 мс слишком быстро —
// команды не успевают, ответы рассинхронизируются и приходит мусор.
final pollIntervalMsProvider = StateProvider<int>(
    (ref) => ref.watch(prefsProvider).getInt("pollIntervalMs") ?? 500);

/// Фабрика транспорта по выбранному типу.
ObdTransport _buildTransport(TransportKind kind, {String? host, int? port}) {
  switch (kind) {
    case TransportKind.ble:
      return BleTransport();
    case TransportKind.wifi:
      return WifiTransport(
        host: host ?? "192.168.0.10",
        port: port ?? 35000,
      );
  }
}

/// Конечный автомат соединения + жизненный цикл ObdService.
class ConnectionController extends StateNotifier<ConnectionUiState> {
  ConnectionController() : super(const ConnectionUiState());

  ObdService? _service;
  ObdService? get service => _service;

  Future<void> _ensurePermissions(TransportKind kind) async {
    if (kind == TransportKind.wifi) return; // Wi-Fi не требует BT-разрешений
    if (Platform.isAndroid || Platform.isIOS) {
      await [
        Permission.bluetoothScan,
        Permission.bluetoothConnect,
        Permission.locationWhenInUse, // нужно для скана на старых Android
      ].request();
    }
  }

  Future<void> connect(
    TransportKind kind, {
    String? wifiHost,
    int? wifiPort,
    Duration interval = const Duration(milliseconds: 200),
  }) async {
    await _start(
      _buildTransport(kind, host: wifiHost, port: wifiPort),
      interval: interval,
      demo: false,
      ensure: () => _ensurePermissions(kind),
    );
  }

  /// Демо-режим: виртуальный адаптер, без железа и без BT-разрешений.
  Future<void> connectDemo({
    Duration interval = const Duration(milliseconds: 200),
  }) async {
    await _start(FakeTransport(), interval: interval, demo: true);
  }

  Future<void> _start(
    ObdTransport transport, {
    required Duration interval,
    required bool demo,
    Future<void> Function()? ensure,
  }) async {
    try {
      state = state.copyWith(link: LinkState.connecting, error: null, demo: demo);
      if (ensure != null) await ensure();

      final service = ObdService(transport);
      _service = service;

      transport.linkState.listen((s) {
        state = state.copyWith(link: s);
        if (s == LinkState.disconnected) _service?.stopPolling();
      });

      await transport.connect();

      state = state.copyWith(link: LinkState.initializing);
      await service.initialize();

      service.startPolling(interval: interval);
      state = state.copyWith(link: LinkState.connected);
    } catch (e) {
      state = state.copyWith(link: LinkState.error, error: e.toString());
    }
  }

  Future<void> disconnect() async {
    await _service?.transport.disconnect();
    await _service?.dispose();
    _service = null;
    state = const ConnectionUiState();
  }
}

class ConnectionUiState {
  final LinkState link;
  final String? error;
  final bool demo;
  const ConnectionUiState({
    this.link = LinkState.disconnected,
    this.error,
    this.demo = false,
  });

  bool get isConnected => link == LinkState.connected;
  bool get isBusy =>
      link == LinkState.connecting ||
      link == LinkState.initializing ||
      link == LinkState.scanning;

  ConnectionUiState copyWith({LinkState? link, String? error, bool? demo}) =>
      ConnectionUiState(
        link: link ?? this.link,
        error: error,
        demo: demo ?? this.demo,
      );
}

final connectionProvider =
    StateNotifierProvider<ConnectionController, ConnectionUiState>(
  (ref) => ConnectionController(),
);

/// Стрим телеметрии для дашборда.
final telemetryProvider = StreamProvider<Telemetry>((ref) {
  // Перестраиваемся при смене состояния линка, иначе после подключения
  // (service создаётся позже) стрим телеметрии не подхватился бы.
  ref.watch(connectionProvider);
  final ctrl = ref.read(connectionProvider.notifier);
  final svc = ctrl.service;
  if (svc == null) return const Stream.empty();
  return svc.telemetry;
});


// ===== палитра и тема приложения =====

/// Единая палитра. Намеренно уходим от «синего» стиля типовых OBD-приложений:
/// тёмный графит + энергичный оранжевый акцент и бирюзовый второстепенный.
class AppColors {
  static const bg = Color(0xFF0C100F);
  static const surface = Color(0xFF171D1C);
  static const surface2 = Color(0xFF212927);
  static const accent = Color(0xFFFF6A2C); // оранжевый
  static const accent2 = Color(0xFF2DD4BF); // бирюзовый
  static const ok = Color(0xFF34D399);
  static const warn = Color(0xFFF5A524);
  static const err = Color(0xFFFF5A5F);
  static const text = Color(0xFFF2F4F3);
  static const textDim = Color(0xFF8A938F);
}

/// Скруглённый прогресс-прибор в новом стиле: толстая дуга со скруглением,
/// крупное число в центре, акцентный «бегунок» вместо классической стрелки.
class GaugeWidget extends StatelessWidget {
  final String label;
  final String unit;
  final double value;
  final double maxValue;
  final Color color;

  const GaugeWidget({
    super.key,
    required this.label,
    required this.unit,
    required this.value,
    required this.maxValue,
    this.color = AppColors.accent,
  });

  @override
  Widget build(BuildContext context) {
    return AspectRatio(
      aspectRatio: 1,
      child: CustomPaint(
        painter: _GaugePainter(value.clamp(0, maxValue), maxValue, color),
        child: Center(
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              const SizedBox(height: 30),
              Text(value.toStringAsFixed(0),
                  style: const TextStyle(
                      color: AppColors.text,
                      fontSize: 32,
                      fontWeight: FontWeight.w800,
                      letterSpacing: -1)),
              Text(unit,
                  style: const TextStyle(color: AppColors.textDim, fontSize: 12)),
              const SizedBox(height: 2),
              Text(label.toUpperCase(),
                  style: TextStyle(
                      color: color,
                      fontSize: 11,
                      letterSpacing: 1.2,
                      fontWeight: FontWeight.w600)),
            ],
          ),
        ),
      ),
    );
  }
}

class _GaugePainter extends CustomPainter {
  final double value;
  final double maxValue;
  final Color color;
  _GaugePainter(this.value, this.maxValue, this.color);

  // дуга-«подкова» снизу: от 130° на 280°
  static const _start = 130 * pi / 180;
  static const _sweep = 280 * pi / 180;

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    final radius = size.width / 2 - 12;
    final rect = Rect.fromCircle(center: center, radius: radius);

    final bg = Paint()
      ..color = AppColors.surface2
      ..style = PaintingStyle.stroke
      ..strokeWidth = 13
      ..strokeCap = StrokeCap.round;
    canvas.drawArc(rect, _start, _sweep, false, bg);

    final fillAngle = _sweep * (value / maxValue).clamp(0, 1);
    final fg = Paint()
      ..shader = SweepGradient(
        startAngle: _start,
        endAngle: _start + _sweep,
        colors: [color.withOpacity(0.55), color],
        transform: GradientRotation(_start),
      ).createShader(rect)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 13
      ..strokeCap = StrokeCap.round;
    canvas.drawArc(rect, _start, fillAngle, false, fg);

    // бегунок на конце заполнения
    final a = _start + fillAngle;
    final knob = Offset(center.dx + radius * cos(a), center.dy + radius * sin(a));
    canvas.drawCircle(knob, 9, Paint()..color = color);
    canvas.drawCircle(knob, 4, Paint()..color = AppColors.bg);
  }

  @override
  bool shouldRepaint(_GaugePainter old) => old.value != value;
}


// ===== общие UI-компоненты новой темы =====

/// Декорация «панели»: тёмная поверхность со скруглением и тонкой обводкой.
BoxDecoration panelDecoration({Color? border}) => BoxDecoration(
      color: AppColors.surface,
      borderRadius: BorderRadius.circular(20),
      border: Border.all(color: border ?? Colors.white.withOpacity(0.06)),
    );

/// Заголовок-секция с цветной чертой слева.
class SectionTitle extends StatelessWidget {
  final String text;
  const SectionTitle(this.text, {super.key});
  @override
  Widget build(BuildContext context) => Padding(
        padding: const EdgeInsets.fromLTRB(4, 8, 4, 10),
        child: Row(children: [
          Container(width: 4, height: 16, decoration: BoxDecoration(
              color: AppColors.accent, borderRadius: BorderRadius.circular(2))),
          const SizedBox(width: 8),
          Text(text.toUpperCase(),
              style: const TextStyle(
                  color: AppColors.textDim,
                  fontSize: 12,
                  letterSpacing: 1.5,
                  fontWeight: FontWeight.w700)),
        ]),
      );
}

/// Маленький индикатор-«пилюля» статуса линка для AppBar.
class _StatusChip extends StatelessWidget {
  final LinkState link;
  const _StatusChip(this.link);

  @override
  Widget build(BuildContext context) {
    final map = {
      LinkState.disconnected: ("офлайн", AppColors.textDim),
      LinkState.scanning: ("поиск", AppColors.warn),
      LinkState.connecting: ("связь", AppColors.warn),
      LinkState.initializing: ("init", AppColors.warn),
      LinkState.connected: ("онлайн", AppColors.ok),
      LinkState.error: ("ошибка", AppColors.err),
    };
    final (text, color) = map[link]!;
    return Container(
      margin: const EdgeInsets.only(right: 14),
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 5),
      decoration: BoxDecoration(
        color: color.withOpacity(0.14),
        borderRadius: BorderRadius.circular(20),
      ),
      child: Row(mainAxisSize: MainAxisSize.min, children: [
        Container(width: 7, height: 7, decoration: BoxDecoration(
            color: color, shape: BoxShape.circle)),
        const SizedBox(width: 6),
        Text(text, style: TextStyle(
            color: color, fontSize: 12, fontWeight: FontWeight.w600)),
      ]),
    );
  }
}

/// Заглушка для экранов, которым нужно активное соединение.
class _NotConnected extends StatelessWidget {
  final String? hint;
  const _NotConnected({this.hint});

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          Container(
            padding: const EdgeInsets.all(22),
            decoration: BoxDecoration(
                color: AppColors.surface, shape: BoxShape.circle),
            child: const Icon(Icons.power_off_rounded,
                color: AppColors.textDim, size: 44),
          ),
          const SizedBox(height: 16),
          const Text("Адаптер не подключён",
              style: TextStyle(
                  color: AppColors.text,
                  fontSize: 17,
                  fontWeight: FontWeight.w600)),
          const SizedBox(height: 6),
          Text(hint ?? "Откройте вкладку «Гараж» и нажмите\n«Подключить» или «Демо-режим».",
              textAlign: TextAlign.center,
              style: const TextStyle(color: AppColors.textDim, fontSize: 13)),
        ],
      ),
    );
  }
}

/// Большая карточка подключения — «сердце» вкладки «Гараж».
class ConnectCard extends ConsumerWidget {
  const ConnectCard({super.key});

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final conn = ref.watch(connectionProvider);
    final elmOk = conn.link == LinkState.initializing || conn.isConnected;
    final ecuOk = conn.isConnected;

    return Container(
      padding: const EdgeInsets.all(18),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(24),
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: conn.isConnected
              ? [AppColors.accent2.withOpacity(0.22), AppColors.surface]
              : [AppColors.surface2, AppColors.surface],
        ),
        border: Border.all(color: Colors.white.withOpacity(0.06)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          Row(children: [
            Icon(conn.isConnected ? Icons.bolt : Icons.bluetooth_searching,
                color: conn.isConnected ? AppColors.accent2 : AppColors.accent, size: 26),
            const SizedBox(width: 10),
            Expanded(
              child: Text(
                conn.isBusy
                    ? "Устанавливаем связь…"
                    : conn.isConnected
                        ? (conn.demo ? "Демо-режим активен" : "Связь с автомобилем")
                        : "Готов к подключению",
                style: const TextStyle(
                    color: AppColors.text, fontSize: 17, fontWeight: FontWeight.w700),
              ),
            ),
          ]),
          const SizedBox(height: 14),
          Row(children: [
            _LinkDot(label: "ELM327", ok: elmOk),
            const SizedBox(width: 10),
            _LinkDot(label: "ЭБУ", ok: ecuOk),
          ]),
          if (conn.error != null) ...[
            const SizedBox(height: 10),
            Text(conn.error!,
                style: const TextStyle(color: AppColors.err, fontSize: 12)),
          ],
          const SizedBox(height: 16),
          Row(children: [
            Expanded(
              flex: 2,
              child: FilledButton(
                style: FilledButton.styleFrom(
                  backgroundColor:
                      conn.isConnected ? AppColors.err : AppColors.accent,
                  foregroundColor: Colors.white,
                  padding: const EdgeInsets.symmetric(vertical: 14),
                  shape: RoundedRectangleBorder(
                      borderRadius: BorderRadius.circular(14)),
                ),
                onPressed: conn.isBusy
                    ? null
                    : () {
                        final ctrl = ref.read(connectionProvider.notifier);
                        if (conn.isConnected) {
                          ctrl.disconnect();
                        } else {
                          ctrl.connect(
                            ref.read(transportKindProvider),
                            wifiHost: ref.read(wifiHostProvider),
                            wifiPort: ref.read(wifiPortProvider),
                            interval: Duration(
                                milliseconds: ref.read(pollIntervalMsProvider)),
                          );
                        }
                      },
                child: Text(conn.isBusy
                    ? "Подключение…"
                    : conn.isConnected
                        ? "Отключить"
                        : "Подключить"),
              ),
            ),
            const SizedBox(width: 10),
            Expanded(
              child: OutlinedButton(
                style: OutlinedButton.styleFrom(
                  foregroundColor: AppColors.accent2,
                  side: const BorderSide(color: AppColors.accent2),
                  padding: const EdgeInsets.symmetric(vertical: 14),
                  shape: RoundedRectangleBorder(
                      borderRadius: BorderRadius.circular(14)),
                ),
                onPressed: conn.isBusy || conn.isConnected
                    ? null
                    : () => ref.read(connectionProvider.notifier).connectDemo(
                          interval: Duration(
                              milliseconds: ref.read(pollIntervalMsProvider)),
                        ),
                child: const Text("Демо"),
              ),
            ),
          ]),
        ],
      ),
    );
  }
}

class _LinkDot extends StatelessWidget {
  final String label;
  final bool ok;
  const _LinkDot({required this.label, required this.ok});
  @override
  Widget build(BuildContext context) {
    final c = ok ? AppColors.ok : AppColors.textDim;
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 7),
      decoration: BoxDecoration(
          color: AppColors.bg.withOpacity(0.4),
          borderRadius: BorderRadius.circular(10)),
      child: Row(mainAxisSize: MainAxisSize.min, children: [
        Icon(ok ? Icons.check_circle : Icons.remove_circle_outline,
            color: c, size: 15),
        const SizedBox(width: 6),
        Text(label, style: TextStyle(color: c, fontSize: 12, fontWeight: FontWeight.w600)),
      ]),
    );
  }
}

/// Карточка одного живого параметра с мини-полосой заполнения.
class MetricCard extends StatelessWidget {
  final String label;
  final String value;
  final String unit;
  final double fraction; // 0..1 для мини-бара
  final Color color;
  final IconData icon;
  const MetricCard({
    super.key,
    required this.label,
    required this.value,
    required this.unit,
    required this.fraction,
    required this.color,
    required this.icon,
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(14),
      decoration: panelDecoration(),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(children: [
            Icon(icon, color: color, size: 18),
            const SizedBox(width: 6),
            Expanded(
              child: Text(label.toUpperCase(),
                  maxLines: 1,
                  overflow: TextOverflow.ellipsis,
                  style: const TextStyle(
                      color: AppColors.textDim,
                      fontSize: 10.5,
                      letterSpacing: 1,
                      fontWeight: FontWeight.w600)),
            ),
          ]),
          const SizedBox(height: 10),
          Row(crossAxisAlignment: CrossAxisAlignment.baseline,
              textBaseline: TextBaseline.alphabetic, children: [
            Text(value,
                style: const TextStyle(
                    color: AppColors.text,
                    fontSize: 26,
                    fontWeight: FontWeight.w800,
                    letterSpacing: -0.5)),
            const SizedBox(width: 4),
            Text(unit, style: const TextStyle(color: AppColors.textDim, fontSize: 12)),
          ]),
          const SizedBox(height: 10),
          ClipRRect(
            borderRadius: BorderRadius.circular(4),
            child: LinearProgressIndicator(
              value: fraction.clamp(0, 1),
              minHeight: 5,
              backgroundColor: AppColors.surface2,
              valueColor: AlwaysStoppedAnimation(color),
            ),
          ),
        ],
      ),
    );
  }
}

/// Строка-«плитка» хаба (Диагностика / Ещё) — список вместо сетки иконок.
class HubTile extends StatelessWidget {
  final IconData icon;
  final String title;
  final String subtitle;
  final Color color;
  final VoidCallback onTap;
  const HubTile({
    super.key,
    required this.icon,
    required this.title,
    required this.subtitle,
    required this.color,
    required this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.only(bottom: 10),
      child: Material(
        color: Colors.transparent,
        child: InkWell(
          borderRadius: BorderRadius.circular(16),
          onTap: onTap,
          child: Container(
            padding: const EdgeInsets.all(14),
            decoration: panelDecoration(),
            child: Row(children: [
              Container(
                width: 42, height: 42,
                decoration: BoxDecoration(
                    color: color.withOpacity(0.15),
                    borderRadius: BorderRadius.circular(12)),
                child: Icon(icon, color: color, size: 22),
              ),
              const SizedBox(width: 14),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(title, style: const TextStyle(
                        color: AppColors.text, fontSize: 15, fontWeight: FontWeight.w600)),
                    const SizedBox(height: 2),
                    Text(subtitle, style: const TextStyle(
                        color: AppColors.textDim, fontSize: 12)),
                  ],
                ),
              ),
              const Icon(Icons.chevron_right, color: AppColors.textDim),
            ]),
          ),
        ),
      ),
    );
  }
}


// ===== вкладки и экраны =====

/// Вкладка «Гараж»: подключение + ключевые живые показатели + сводка ошибок.
class _DashboardTab extends ConsumerWidget {
  const _DashboardTab();

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final conn = ref.watch(connectionProvider);
    final tele = ref.watch(telemetryProvider);
    final t = tele.asData?.value ?? const Telemetry();

    return ListView(
      padding: const EdgeInsets.fromLTRB(16, 16, 16, 24),
      children: [
        const ConnectCard(),
        const SizedBox(height: 16),
        if (conn.isConnected) ...[
          const SectionTitle("Ключевые показатели"),
          GridView.count(
            crossAxisCount: 2,
            shrinkWrap: true,
            physics: const NeverScrollableScrollPhysics(),
            mainAxisSpacing: 12,
            crossAxisSpacing: 12,
            childAspectRatio: 1.45,
            children: [
              MetricCard(label: "Скорость", value: "${t.speed ?? 0}", unit: "км/ч",
                  fraction: (t.speed ?? 0) / 240, color: AppColors.accent2, icon: Icons.speed),
              MetricCard(label: "Обороты", value: "${t.rpm ?? 0}", unit: "об/мин",
                  fraction: (t.rpm ?? 0) / 8000, color: AppColors.accent, icon: Icons.autorenew),
              MetricCard(label: "Темп. ОЖ", value: "${t.coolant ?? 0}", unit: "°C",
                  fraction: (t.coolant ?? 0) / 130, color: AppColors.warn, icon: Icons.thermostat),
              MetricCard(label: "Напряжение", value: (t.voltage ?? 0).toStringAsFixed(1), unit: "В",
                  fraction: (t.voltage ?? 0) / 15, color: AppColors.ok, icon: Icons.battery_charging_full),
            ],
          ),
          const SizedBox(height: 8),
          const SectionTitle("Диагностика"),
          const _DtcPanel(),
        ] else
          Padding(
            padding: const EdgeInsets.only(top: 40),
            child: _NotConnected(
                hint: "Нажмите «Подключить» для связи с адаптером\nили «Демо» для просмотра без авто."),
          ),
      ],
    );
  }
}

/// Экран «Показатели» — круговые приборы во весь экран (открывается из «Ещё»).
class GaugesScreen extends ConsumerWidget {
  const GaugesScreen({super.key});

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final conn = ref.watch(connectionProvider);
    return Scaffold(
      backgroundColor: const Color(0xFF0C100F),
      appBar: AppBar(
        backgroundColor: const Color(0xFF171D1C),
        title: const Text("Показатели"),
        actions: [_StatusChip(conn.link)],
      ),
      body: Padding(
        padding: const EdgeInsets.all(16),
        child: conn.isConnected ? const _GaugesGrid() : const _NotConnected(),
      ),
    );
  }
}

class _GaugesGrid extends ConsumerWidget {
  const _GaugesGrid();

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final tele = ref.watch(telemetryProvider);

    return tele.when(
      data: (t) => GridView.count(
        crossAxisCount: 2,
        mainAxisSpacing: 16,
        crossAxisSpacing: 16,
        children: [
          GaugeWidget(
              label: "Обороты",
              unit: "об/мин",
              value: (t.rpm ?? 0).toDouble(),
              maxValue: 8000,
              color: AppColors.accent),
          GaugeWidget(
              label: "Скорость",
              unit: "км/ч",
              value: (t.speed ?? 0).toDouble(),
              maxValue: 240,
              color: AppColors.accent2),
          GaugeWidget(
              label: "Темп. ОЖ",
              unit: "°C",
              value: (t.coolant ?? 0).toDouble(),
              maxValue: 130,
              color: AppColors.warn),
          GaugeWidget(
              label: "Нагрузка",
              unit: "%",
              value: t.load ?? 0,
              maxValue: 100,
              color: AppColors.ok),
          GaugeWidget(
              label: "Наддув (MAP)",
              unit: "кПа",
              value: (t.map ?? 0).toDouble(),
              maxValue: 250,
              color: AppColors.accent2),
          GaugeWidget(
              label: "Топливо",
              unit: "%",
              value: t.fuelLevel ?? 0,
              maxValue: 100,
              color: AppColors.warn),
          GaugeWidget(
              label: "Расход",
              unit: "л/ч",
              value: t.fuelRate ?? 0,
              maxValue: 30,
              color: AppColors.accent),
          GaugeWidget(
              label: "Темп. впуска",
              unit: "°C",
              value: (t.intakeTemp ?? 0).toDouble(),
              maxValue: 90,
              color: AppColors.ok),
        ],
      ),
      loading: () => const Center(child: CircularProgressIndicator()),
      error: (e, _) => Center(
        child: Text("Нет данных: $e",
            style: const TextStyle(color: Colors.redAccent)),
      ),
    );
  }
}

/// Компактная карточка на дашборде — открывает полный экран «Ошибки (DTC)».
class _DtcPanel extends StatelessWidget {
  const _DtcPanel();

  @override
  Widget build(BuildContext context) {
    return Material(
      color: Colors.transparent,
      child: InkWell(
        borderRadius: BorderRadius.circular(20),
        onTap: () => Navigator.of(context)
            .push(MaterialPageRoute(builder: (_) => const DtcScreen())),
        child: Container(
          padding: const EdgeInsets.all(16),
          decoration: panelDecoration(),
          child: Row(children: [
            Container(
              width: 44,
              height: 44,
              decoration: BoxDecoration(
                  color: AppColors.err.withOpacity(0.15),
                  borderRadius: BorderRadius.circular(12)),
              child: const Icon(Icons.error_outline, color: AppColors.err, size: 22),
            ),
            const SizedBox(width: 14),
            const Expanded(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text("Коды ошибок (DTC)",
                      style: TextStyle(
                          color: AppColors.text,
                          fontSize: 15,
                          fontWeight: FontWeight.w600)),
                  SizedBox(height: 2),
                  Text("Сохранённые, ожидающие и постоянные + расшифровка",
                      style: TextStyle(color: AppColors.textDim, fontSize: 12)),
                ],
              ),
            ),
            const Icon(Icons.chevron_right, color: AppColors.textDim),
          ]),
        ),
      ),
    );
  }
}

// ===== модель «Мои автомобили» =====

class Vehicle {
  final String name;
  final String vin;
  const Vehicle(this.name, this.vin);

  Map<String, dynamic> toJson() => {"name": name, "vin": vin};
  factory Vehicle.fromJson(Map<String, dynamic> j) =>
      Vehicle((j["name"] ?? "") as String, (j["vin"] ?? "") as String);
}

/// Список автомобилей с сохранением в SharedPreferences (ключ "vehicles").
class VehiclesNotifier extends StateNotifier<List<Vehicle>> {
  final SharedPreferences _prefs;
  static const _key = "vehicles";

  VehiclesNotifier(this._prefs) : super(_load(_prefs));

  static List<Vehicle> _load(SharedPreferences p) {
    final raw = p.getString(_key);
    if (raw == null) return const [Vehicle("Мой автомобиль", "")];
    try {
      final list = (jsonDecode(raw) as List)
          .map((e) => Vehicle.fromJson(e as Map<String, dynamic>))
          .toList();
      return list;
    } catch (_) {
      return const [Vehicle("Мой автомобиль", "")];
    }
  }

  void _persist() =>
      _prefs.setString(_key, jsonEncode(state.map((v) => v.toJson()).toList()));

  void add(Vehicle v) {
    state = [...state, v];
    _persist();
  }

  void removeAt(int i) {
    state = [...state]..removeAt(i);
    _persist();
  }
}

final vehiclesProvider = StateNotifierProvider<VehiclesNotifier, List<Vehicle>>(
    (ref) => VehiclesNotifier(ref.watch(prefsProvider)));


// ===== журнал проверок ошибок (история DTC) =====

class DtcLogEntry {
  final DateTime time;
  final bool milOn;
  final List<String> stored;
  final List<String> pending;
  final List<String> permanent;
  const DtcLogEntry({
    required this.time,
    required this.milOn,
    required this.stored,
    required this.pending,
    required this.permanent,
  });

  int get total => stored.length + pending.length + permanent.length;

  Map<String, dynamic> toJson() => {
        "t": time.millisecondsSinceEpoch,
        "mil": milOn,
        "s": stored,
        "p": pending,
        "pm": permanent,
      };

  factory DtcLogEntry.fromJson(Map<String, dynamic> j) => DtcLogEntry(
        time: DateTime.fromMillisecondsSinceEpoch((j["t"] ?? 0) as int),
        milOn: (j["mil"] ?? false) as bool,
        stored: ((j["s"] ?? []) as List).cast<String>(),
        pending: ((j["p"] ?? []) as List).cast<String>(),
        permanent: ((j["pm"] ?? []) as List).cast<String>(),
      );
}

/// История проверок DTC с сохранением в SharedPreferences (ключ "dtc_logs").
/// Логи остаются между сеансами и видны без подключения.
class DtcLogNotifier extends StateNotifier<List<DtcLogEntry>> {
  final SharedPreferences _prefs;
  static const _key = "dtc_logs";
  static const _max = 50;

  DtcLogNotifier(this._prefs) : super(_load(_prefs));

  static List<DtcLogEntry> _load(SharedPreferences p) {
    final raw = p.getString(_key);
    if (raw == null) return const [];
    try {
      return (jsonDecode(raw) as List)
          .map((e) => DtcLogEntry.fromJson(e as Map<String, dynamic>))
          .toList();
    } catch (_) {
      return const [];
    }
  }

  void add(DtcLogEntry e) {
    state = [e, ...state].take(_max).toList();
    _prefs.setString(_key, jsonEncode(state.map((x) => x.toJson()).toList()));
  }

  void clear() {
    state = const [];
    _prefs.remove(_key);
  }
}

final dtcLogProvider =
    StateNotifierProvider<DtcLogNotifier, List<DtcLogEntry>>(
        (ref) => DtcLogNotifier(ref.watch(prefsProvider)));


// ===== корневая оболочка с нижней навигацией =====

class RootShell extends ConsumerStatefulWidget {
  const RootShell({super.key});
  @override
  ConsumerState<RootShell> createState() => _RootShellState();
}

class _RootShellState extends ConsumerState<RootShell> {
  int _index = 0;

  static const _titles = ["Гараж", "Датчики", "Диагностика", "Ещё"];

  @override
  Widget build(BuildContext context) {
    final conn = ref.watch(connectionProvider);

    return Scaffold(
      backgroundColor: AppColors.bg,
      appBar: AppBar(
        backgroundColor: AppColors.bg,
        elevation: 0,
        titleSpacing: 20,
        title: Row(children: [
          Text(_index == 0 ? "Revoscan" : _titles[_index],
              style: const TextStyle(
                  fontWeight: FontWeight.w800, fontSize: 20, letterSpacing: -0.5)),
        ]),
        actions: [_StatusChip(conn.link)],
      ),
      body: IndexedStack(
        index: _index,
        children: const [
          _DashboardTab(),
          _SensorsTab(),
          _DiagnosticsTab(),
          _MoreTab(),
        ],
      ),
      bottomNavigationBar: NavigationBar(
        selectedIndex: _index,
        onDestinationSelected: (i) => setState(() => _index = i),
        backgroundColor: AppColors.surface,
        indicatorColor: AppColors.accent.withOpacity(0.22),
        destinations: const [
          NavigationDestination(
              icon: Icon(Icons.garage_outlined),
              selectedIcon: Icon(Icons.garage, color: AppColors.accent),
              label: "Гараж"),
          NavigationDestination(
              icon: Icon(Icons.sensors_outlined),
              selectedIcon: Icon(Icons.sensors, color: AppColors.accent),
              label: "Датчики"),
          NavigationDestination(
              icon: Icon(Icons.troubleshoot_outlined),
              selectedIcon: Icon(Icons.troubleshoot, color: AppColors.accent),
              label: "Диагностика"),
          NavigationDestination(
              icon: Icon(Icons.apps_outlined),
              selectedIcon: Icon(Icons.apps, color: AppColors.accent),
              label: "Ещё"),
        ],
      ),
    );
  }
}

/// Вкладка «Датчики» — компактный живой список всех параметров.
class _SensorsTab extends ConsumerWidget {
  const _SensorsTab();

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final conn = ref.watch(connectionProvider);
    final tele = ref.watch(telemetryProvider);
    if (!conn.isConnected) return const _NotConnected();
    final t = tele.asData?.value ?? const Telemetry();
    final rows = _sensorRows(t);

    return ListView.separated(
      padding: const EdgeInsets.all(16),
      itemCount: rows.length,
      separatorBuilder: (_, __) => const SizedBox(height: 8),
      itemBuilder: (_, i) {
        final (label, value, unit) = rows[i];
        return Container(
          padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
          decoration: panelDecoration(),
          child: Row(children: [
            Expanded(
                child: Text(label,
                    style: const TextStyle(color: AppColors.text, fontSize: 14))),
            Text(value,
                style: const TextStyle(
                    color: AppColors.accent,
                    fontSize: 18,
                    fontWeight: FontWeight.w700)),
            const SizedBox(width: 4),
            Text(unit, style: const TextStyle(color: AppColors.textDim, fontSize: 12)),
          ]),
        );
      },
    );
  }
}

/// Вкладка «Диагностика» — список разделов вместо сетки иконок.
class _DiagnosticsTab extends ConsumerWidget {
  const _DiagnosticsTab();

  void _open(BuildContext c, Widget screen, bool needConn, bool connected) {
    if (needConn && !connected) {
      ScaffoldMessenger.of(c).showSnackBar(const SnackBar(
          content: Text("Нужно подключение. Откройте «Гараж» → «Подключить» или «Демо».")));
      return;
    }
    Navigator.of(c).push(MaterialPageRoute(builder: (_) => screen));
  }

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final connected = ref.watch(connectionProvider).isConnected;
    return ListView(
      padding: const EdgeInsets.all(16),
      children: [
        HubTile(icon: Icons.error_outline, color: AppColors.err,
            title: "Ошибки (DTC)", subtitle: "Чтение и сброс кодов неисправностей",
            onTap: () => _open(context, const DtcScreen(), true, connected)),
        HubTile(icon: Icons.ac_unit, color: AppColors.accent2,
            title: "Стоп-кадр", subtitle: "Параметры в момент ошибки",
            onTap: () => _open(context, const FreezeFrameScreen(), true, connected)),
        HubTile(icon: Icons.eco, color: AppColors.ok,
            title: "Тесты на выбросы", subtitle: "Готовность бортовых мониторов",
            onTap: () => _open(context, const EmissionsScreen(), true, connected)),
        HubTile(icon: Icons.badge_outlined, color: AppColors.accent,
            title: "Идентификаторы ЭБУ", subtitle: "VIN и калибровка",
            onTap: () => _open(context, const EcuIdScreen(), true, connected)),
      ],
    );
  }
}

/// Вкладка «Ещё» — приборы, инструменты, гараж, настройки.
class _MoreTab extends ConsumerWidget {
  const _MoreTab();

  void _open(BuildContext c, Widget screen, bool needConn, bool connected) {
    if (needConn && !connected) {
      ScaffoldMessenger.of(c).showSnackBar(const SnackBar(
          content: Text("Нужно подключение. Откройте «Гараж» → «Подключить» или «Демо».")));
      return;
    }
    Navigator.of(c).push(MaterialPageRoute(builder: (_) => screen));
  }

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final connected = ref.watch(connectionProvider).isConnected;
    return ListView(
      padding: const EdgeInsets.all(16),
      children: [
        const SectionTitle("Приборы"),
        HubTile(icon: Icons.donut_large, color: AppColors.accent,
            title: "Показатели", subtitle: "Круговые приборы",
            onTap: () => _open(context, const GaugesScreen(), true, connected)),
        HubTile(icon: Icons.memory, color: AppColors.accent2,
            title: "Мониторинг ЭБУ", subtitle: "Живой поток параметров",
            onTap: () => _open(context, const SensorListScreen(title: "Мониторинг ЭБУ"), true, connected)),
        const SectionTitle("Инструменты"),
        HubTile(icon: Icons.timer_outlined, color: AppColors.warn,
            title: "Замер разгона", subtitle: "0–60 и 0–100 км/ч",
            onTap: () => _open(context, const AccelerationScreen(), true, connected)),
        HubTile(icon: Icons.save_alt, color: AppColors.accent2,
            title: "Запись данных", subtitle: "Лог телеметрии и экспорт CSV",
            onTap: () => _open(context, const DataLoggingScreen(), true, connected)),
        HubTile(icon: Icons.bar_chart, color: AppColors.ok,
            title: "Статистика", subtitle: "Мин / тек / макс",
            onTap: () => _open(context, const StatisticsScreen(), true, connected)),
        const SectionTitle("Настройки"),
        HubTile(icon: Icons.directions_car_filled, color: AppColors.accent,
            title: "Мои автомобили", subtitle: "Профили машин",
            onTap: () => _open(context, const MyCarsScreen(), false, connected)),
        HubTile(icon: Icons.tune, color: AppColors.textDim,
            title: "Настройки", subtitle: "Адаптер, Wi-Fi, опрос",
            onTap: () => _open(context, const SettingsScreen(), false, connected)),
      ],
    );
  }
}


// ===== экран «Все датчики» / «Мониторинг ЭБУ» =====

/// Строки телеметрии в виде (название, значение, единица).
List<(String, String, String)> _sensorRows(Telemetry t) {
  String n(num? v, [int d = 0]) => v == null ? "—" : v.toStringAsFixed(d);
  return [
    ("Обороты", n(t.rpm), "об/мин"),
    ("Скорость", n(t.speed), "км/ч"),
    ("Темп. ОЖ", n(t.coolant), "°C"),
    ("Нагрузка", n(t.load, 1), "%"),
    ("Напряжение", n(t.voltage, 1), "В"),
    ("Темп. впуска", n(t.intakeTemp), "°C"),
    ("Уровень топлива", n(t.fuelLevel, 1), "%"),
    ("MAF", n(t.maf, 1), "г/с"),
    ("Наддув (MAP)", n(t.map), "кПа"),
    ("Опереж. зажигания", n(t.timing, 1), "°"),
    ("Темп. за бортом", n(t.ambient), "°C"),
    ("Расход топлива", n(t.fuelRate, 1), "л/ч"),
  ];
}

class SensorListScreen extends ConsumerWidget {
  final String title;
  const SensorListScreen({super.key, required this.title});

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final conn = ref.watch(connectionProvider);
    final tele = ref.watch(telemetryProvider);

    return Scaffold(
      backgroundColor: const Color(0xFF0C100F),
      appBar: AppBar(
        backgroundColor: const Color(0xFF171D1C),
        title: Text(title),
        actions: [_StatusChip(conn.link)],
      ),
      body: !conn.isConnected
          ? const _NotConnected()
          : tele.when(
              data: (t) {
                final rows = _sensorRows(t);
                return ListView.separated(
                  itemCount: rows.length,
                  separatorBuilder: (_, __) =>
                      const Divider(height: 1, color: Colors.white10),
                  itemBuilder: (_, i) {
                    final (label, value, unit) = rows[i];
                    return ListTile(
                      dense: true,
                      title: Text(label,
                          style: const TextStyle(color: Colors.white70)),
                      trailing: Text("$value $unit",
                          style: const TextStyle(
                              color: Colors.cyanAccent,
                              fontSize: 16,
                              fontWeight: FontWeight.w600)),
                    );
                  },
                );
              },
              loading: () => const Center(child: CircularProgressIndicator()),
              error: (e, _) => Center(
                  child: Text("Нет данных: $e",
                      style: const TextStyle(color: Colors.redAccent))),
            ),
    );
  }
}


// ===== экран «Ошибки (DTC)» =====

class DtcScreen extends ConsumerStatefulWidget {
  const DtcScreen({super.key});
  @override
  ConsumerState<DtcScreen> createState() => _DtcScreenState();
}

class _DtcScreenState extends ConsumerState<DtcScreen> {
  Readiness? _readiness;
  List<String> _stored = [];
  List<String> _pending = [];
  List<String> _permanent = [];
  bool _loading = false;
  bool _loaded = false;
  String? _error;
  String? _progress; // текущий шаг сканирования

  ObdService? get _svc => ref.read(connectionProvider.notifier).service;

  void _step(String s) {
    if (mounted) setState(() => _progress = s);
  }

  Future<void> _scan() async {
    final svc = _svc;
    if (svc == null) return;

    // Ставим опрос телеметрии на паузу и чистим очередь, иначе команды чтения
    // ошибок встают в хвост постоянного потока опроса и могут ждать очень долго.
    final wasPolling = svc.isPolling;
    svc.stopPolling();
    svc.clearQueue();

    setState(() {
      _loading = true;
      _loaded = true; // показываем секции по мере поступления
      _error = null;
      _readiness = null;
      _stored = [];
      _pending = [];
      _permanent = [];
    });

    try {
      _step("Проверяю мониторы (Check Engine)…");
      try {
        _readiness = await svc.readReadiness();
        if (mounted) setState(() {});
      } catch (_) {/* не критично */}

      _step("Читаю сохранённые коды…");
      try {
        final s = await svc.readDtc();
        if (mounted) setState(() => _stored = s);
      } catch (e) {
        _step("Сохранённые коды: ошибка чтения");
      }

      _step("Читаю ожидающие коды…");
      try {
        final p = await svc.readPendingDtc();
        if (mounted) setState(() => _pending = p);
      } catch (_) {/* режим 07 поддерживают не все ЭБУ */}

      _step("Читаю постоянные коды…");
      try {
        final pm = await svc.readPermanentDtc();
        if (mounted) setState(() => _permanent = pm);
      } catch (_) {/* режим 0A поддерживают не все ЭБУ */}

      // Сохраняем результат в журнал, если связь реально отработала.
      if (_readiness != null ||
          _stored.isNotEmpty ||
          _pending.isNotEmpty ||
          _permanent.isNotEmpty) {
        ref.read(dtcLogProvider.notifier).add(DtcLogEntry(
              time: DateTime.now(),
              milOn: _readiness?.milOn ?? false,
              stored: _stored,
              pending: _pending,
              permanent: _permanent,
            ));
      }
    } catch (e) {
      if (mounted) setState(() => _error = "$e");
    } finally {
      if (wasPolling) svc.resumePolling();
      if (mounted) {
        setState(() {
          _loading = false;
          _progress = null;
        });
      }
    }
  }

  Future<void> _clear() async {
    final svc = _svc;
    if (svc == null) return;
    final ok = await showDialog<bool>(
      context: context,
      builder: (_) => AlertDialog(
        backgroundColor: AppColors.surface,
        title: const Text("Сбросить ошибки?", style: TextStyle(color: AppColors.text)),
        content: const Text(
            "Check Engine погаснет, стоп-кадр будет стёрт, а мониторы выбросов "
            "сбросятся в «не готов». Постоянные коды (Mode 0A) так не стираются. "
            "Автомобиль должен стоять (V=0). Продолжить?",
            style: TextStyle(color: AppColors.textDim)),
        actions: [
          TextButton(onPressed: () => Navigator.pop(context, false), child: const Text("Отмена")),
          FilledButton(
              style: FilledButton.styleFrom(backgroundColor: AppColors.err),
              onPressed: () => Navigator.pop(context, true),
              child: const Text("Сбросить")),
        ],
      ),
    );
    if (ok != true) return;

    // Сброс — запись в шину; так же убираем фон опроса, чтобы команда
    // не ждала в очереди телеметрии.
    final wasPolling = svc.isPolling;
    svc.stopPolling();
    svc.clearQueue();
    setState(() {
      _loading = true;
      _progress = "Отправляю команду сброса…";
    });
    try {
      await svc.clearDtc(userConfirmed: true);
      if (wasPolling) svc.resumePolling();
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
            const SnackBar(content: Text("Сброс выполнен — перечитываю коды")));
      }
      await _scan(); // покажет актуальное состояние пошагово
    } catch (e) {
      if (wasPolling) svc.resumePolling();
      if (mounted) {
        setState(() {
          _loading = false;
          _progress = null;
        });
        ScaffoldMessenger.of(context).showSnackBar(
            SnackBar(content: Text("$e"), backgroundColor: AppColors.err));
      }
    }
  }

  void _showDetail(String code, String kind, Color kindColor) {
    final info = DtcCatalog.describe(code);
    showModalBottomSheet(
      context: context,
      backgroundColor: AppColors.surface,
      shape: const RoundedRectangleBorder(
          borderRadius: BorderRadius.vertical(top: Radius.circular(22))),
      builder: (_) => Padding(
        padding: const EdgeInsets.fromLTRB(20, 18, 20, 28),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(children: [
              Text(info.code,
                  style: const TextStyle(
                      color: AppColors.text, fontSize: 26, fontWeight: FontWeight.w800)),
              const Spacer(),
              IconButton(
                icon: const Icon(Icons.copy, color: AppColors.textDim, size: 20),
                onPressed: () {
                  Clipboard.setData(ClipboardData(text: info.code));
                  ScaffoldMessenger.of(context).showSnackBar(
                      const SnackBar(content: Text("Код скопирован")));
                },
              ),
            ]),
            const SizedBox(height: 6),
            Wrap(spacing: 8, runSpacing: 8, children: [
              _tag(kind, kindColor),
              _tag(info.system, AppColors.accent2),
              _tag(info.generic ? "Общий (SAE)" : "Производителя", AppColors.textDim),
            ]),
            const SizedBox(height: 16),
            Text(info.description,
                style: const TextStyle(color: AppColors.text, fontSize: 15, height: 1.4)),
            const SizedBox(height: 18),
            SizedBox(
              width: double.infinity,
              child: OutlinedButton.icon(
                icon: const Icon(Icons.search),
                label: const Text("Скопировать код для поиска"),
                style: OutlinedButton.styleFrom(foregroundColor: AppColors.accent),
                onPressed: () {
                  Clipboard.setData(ClipboardData(text: info.code));
                  Navigator.pop(context);
                  ScaffoldMessenger.of(context).showSnackBar(
                      const SnackBar(content: Text("Код скопирован — вставьте в поиск")));
                },
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _tag(String text, Color color) => Container(
        padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 5),
        decoration: BoxDecoration(
            color: color.withOpacity(0.16), borderRadius: BorderRadius.circular(8)),
        child: Text(text,
            style: TextStyle(color: color, fontSize: 12, fontWeight: FontWeight.w600)),
      );

  @override
  Widget build(BuildContext context) {
    final conn = ref.watch(connectionProvider);
    final total = _stored.length + _pending.length + _permanent.length;
    return Scaffold(
      backgroundColor: const Color(0xFF0C100F),
      appBar: AppBar(
        backgroundColor: const Color(0xFF171D1C),
        title: const Text("Ошибки (DTC)"),
        actions: [
          IconButton(
              icon: const Icon(Icons.history),
              tooltip: "Журнал проверок",
              onPressed: () => Navigator.of(context).push(
                  MaterialPageRoute(builder: (_) => const DtcHistoryScreen()))),
          if (conn.isConnected && _loaded)
            IconButton(
                icon: const Icon(Icons.refresh),
                tooltip: "Пересканировать",
                onPressed: _loading ? null : _scan),
          _StatusChip(conn.link),
        ],
      ),
      body: !conn.isConnected
          ? const _NotConnected()
          : ListView(
              padding: const EdgeInsets.all(16),
              children: [
                _summaryCard(total),
                if (_error != null) ...[
                  const SizedBox(height: 12),
                  Text(_error!, style: const TextStyle(color: AppColors.err)),
                ],
                if (_loading) ...[
                  const SizedBox(height: 14),
                  Container(
                    padding: const EdgeInsets.all(14),
                    decoration: panelDecoration(),
                    child: Row(children: [
                      const SizedBox(
                        width: 20,
                        height: 20,
                        child: CircularProgressIndicator(
                            strokeWidth: 2.4, color: AppColors.accent),
                      ),
                      const SizedBox(width: 14),
                      Expanded(
                        child: Text(_progress ?? "Сканирую…",
                            style: const TextStyle(
                                color: AppColors.text, fontSize: 13.5)),
                      ),
                    ]),
                  ),
                ],
                if (_loaded) ...[
                  _section("Сохранённые", _stored, AppColors.err,
                      "Подтверждённые коды — горит Check Engine"),
                  _section("Ожидающие", _pending, AppColors.warn,
                      "Замечены, но ещё не подтверждены"),
                  _section("Постоянные", _permanent, AppColors.accent2,
                      "Сканером не стираются, гаснут сами"),
                  if (total == 0 && !_loading)
                    Padding(
                      padding: const EdgeInsets.only(top: 24),
                      child: Column(children: const [
                        Icon(Icons.verified, color: AppColors.ok, size: 48),
                        SizedBox(height: 10),
                        Text("Ошибок не найдено",
                            style: TextStyle(color: AppColors.text, fontSize: 16)),
                      ]),
                    ),
                ],
              ],
            ),
      bottomNavigationBar: !conn.isConnected
          ? null
          : SafeArea(
              child: Padding(
                padding: const EdgeInsets.fromLTRB(16, 8, 16, 16),
                child: Row(children: [
                  Expanded(
                    flex: 2,
                    child: FilledButton.icon(
                      style: FilledButton.styleFrom(
                        backgroundColor: AppColors.accent,
                        padding: const EdgeInsets.symmetric(vertical: 14),
                      ),
                      icon: const Icon(Icons.search),
                      label: Text(_loaded ? "Пересканировать" : "Сканировать ошибки"),
                      onPressed: _loading ? null : _scan,
                    ),
                  ),
                  const SizedBox(width: 12),
                  Expanded(
                    child: OutlinedButton.icon(
                      style: OutlinedButton.styleFrom(
                        foregroundColor: AppColors.err,
                        side: const BorderSide(color: AppColors.err),
                        padding: const EdgeInsets.symmetric(vertical: 14),
                      ),
                      icon: const Icon(Icons.delete_outline),
                      label: const Text("Сброс"),
                      onPressed: _stored.isEmpty || _loading ? null : _clear,
                    ),
                  ),
                ]),
              ),
            ),
    );
  }

  Widget _summaryCard(int total) {
    final known = _readiness != null;
    final mil = _readiness?.milOn ?? false;
    final showAlert = known && mil;
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: panelDecoration(
          border: (_loaded && !_loading && total > 0)
              ? AppColors.err.withOpacity(0.4)
              : null),
      child: Row(children: [
        Container(
          width: 54,
          height: 54,
          decoration: BoxDecoration(
            color: (showAlert ? AppColors.err : AppColors.ok).withOpacity(0.15),
            shape: BoxShape.circle,
          ),
          child: Icon(
              _loading
                  ? Icons.hourglass_top
                  : showAlert
                      ? Icons.warning_amber_rounded
                      : Icons.check_circle,
              color: showAlert ? AppColors.err : AppColors.ok,
              size: 28),
        ),
        const SizedBox(width: 14),
        Expanded(
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(
                _loading
                    ? "Сканирование…"
                    : !_loaded
                        ? "Готов к сканированию"
                        : !known
                            ? "Сканирование завершено"
                            : mil
                                ? "Check Engine горит"
                                : "Check Engine не горит",
                style: const TextStyle(
                    color: AppColors.text, fontSize: 16, fontWeight: FontWeight.w700),
              ),
              const SizedBox(height: 2),
              Text(
                !_loaded
                    ? "Нажмите «Сканировать ошибки»"
                    : "Сохранённых: ${_stored.length} • ожидающих: ${_pending.length} • постоянных: ${_permanent.length}",
                style: const TextStyle(color: AppColors.textDim, fontSize: 12.5),
              ),
            ],
          ),
        ),
      ]),
    );
  }

  Widget _section(String title, List<String> codes, Color color, String hint) {
    if (codes.isEmpty) return const SizedBox.shrink();
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        const SizedBox(height: 18),
        Row(children: [
          SectionTitle("$title (${codes.length})"),
        ]),
        Padding(
          padding: const EdgeInsets.only(left: 12, bottom: 6),
          child: Text(hint, style: const TextStyle(color: AppColors.textDim, fontSize: 11.5)),
        ),
        ...codes.map((c) {
          final info = DtcCatalog.describe(c);
          return Padding(
            padding: const EdgeInsets.only(bottom: 8),
            child: Material(
              color: Colors.transparent,
              child: InkWell(
                borderRadius: BorderRadius.circular(14),
                onTap: () => _showDetail(c, title, color),
                child: Container(
                  padding: const EdgeInsets.all(14),
                  decoration: panelDecoration(),
                  child: Row(children: [
                    Container(
                      width: 8,
                      height: 38,
                      decoration: BoxDecoration(
                          color: color, borderRadius: BorderRadius.circular(4)),
                    ),
                    const SizedBox(width: 12),
                    Expanded(
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Text(info.code,
                              style: const TextStyle(
                                  color: AppColors.text,
                                  fontSize: 16,
                                  fontWeight: FontWeight.w700)),
                          const SizedBox(height: 2),
                          Text(info.description,
                              maxLines: 2,
                              overflow: TextOverflow.ellipsis,
                              style: const TextStyle(
                                  color: AppColors.textDim, fontSize: 12.5)),
                        ],
                      ),
                    ),
                    const Icon(Icons.chevron_right, color: AppColors.textDim),
                  ]),
                ),
              ),
            ),
          );
        }),
      ],
    );
  }
}


// ===== экран «Журнал проверок ошибок» =====

class DtcHistoryScreen extends ConsumerWidget {
  const DtcHistoryScreen({super.key});

  String _fmtDate(DateTime t) {
    String two(int n) => n.toString().padLeft(2, '0');
    return "${two(t.day)}.${two(t.month)}.${t.year} ${two(t.hour)}:${two(t.minute)}";
  }

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final logs = ref.watch(dtcLogProvider);
    return Scaffold(
      backgroundColor: const Color(0xFF0C100F),
      appBar: AppBar(
        backgroundColor: const Color(0xFF171D1C),
        title: const Text("Журнал проверок"),
        actions: [
          if (logs.isNotEmpty)
            IconButton(
              icon: const Icon(Icons.delete_sweep),
              tooltip: "Очистить журнал",
              onPressed: () async {
                final ok = await showDialog<bool>(
                  context: context,
                  builder: (_) => AlertDialog(
                    backgroundColor: AppColors.surface,
                    title: const Text("Очистить журнал?",
                        style: TextStyle(color: AppColors.text)),
                    content: const Text("Вся сохранённая история проверок будет удалена.",
                        style: TextStyle(color: AppColors.textDim)),
                    actions: [
                      TextButton(
                          onPressed: () => Navigator.pop(context, false),
                          child: const Text("Отмена")),
                      FilledButton(
                          style: FilledButton.styleFrom(backgroundColor: AppColors.err),
                          onPressed: () => Navigator.pop(context, true),
                          child: const Text("Очистить")),
                    ],
                  ),
                );
                if (ok == true) ref.read(dtcLogProvider.notifier).clear();
              },
            ),
        ],
      ),
      body: logs.isEmpty
          ? const Center(
              child: Text("Пока нет сохранённых проверок",
                  style: TextStyle(color: AppColors.textDim)))
          : ListView.builder(
              padding: const EdgeInsets.all(16),
              itemCount: logs.length,
              itemBuilder: (_, i) {
                final e = logs[i];
                final all = [
                  ...e.stored.map((c) => (c, AppColors.err)),
                  ...e.pending.map((c) => (c, AppColors.warn)),
                  ...e.permanent.map((c) => (c, AppColors.accent2)),
                ];
                return Container(
                  margin: const EdgeInsets.only(bottom: 10),
                  padding: const EdgeInsets.all(14),
                  decoration: panelDecoration(),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Row(children: [
                        Icon(e.milOn ? Icons.warning_amber_rounded : Icons.check_circle,
                            color: e.milOn ? AppColors.err : AppColors.ok, size: 18),
                        const SizedBox(width: 8),
                        Text(_fmtDate(e.time),
                            style: const TextStyle(
                                color: AppColors.text,
                                fontSize: 14,
                                fontWeight: FontWeight.w600)),
                        const Spacer(),
                        Text(e.total == 0 ? "чисто" : "кодов: ${e.total}",
                            style: TextStyle(
                                color: e.total == 0 ? AppColors.ok : AppColors.err,
                                fontSize: 12)),
                      ]),
                      if (all.isNotEmpty) ...[
                        const SizedBox(height: 10),
                        Wrap(
                          spacing: 8,
                          runSpacing: 8,
                          children: all
                              .map((p) => Container(
                                    padding: const EdgeInsets.symmetric(
                                        horizontal: 10, vertical: 5),
                                    decoration: BoxDecoration(
                                        color: p.$2.withOpacity(0.16),
                                        borderRadius: BorderRadius.circular(8)),
                                    child: Text(p.$1,
                                        style: TextStyle(
                                            color: p.$2,
                                            fontSize: 12,
                                            fontWeight: FontWeight.w600)),
                                  ))
                              .toList(),
                        ),
                      ],
                    ],
                  ),
                );
              },
            ),
    );
  }
}


// ===== экран «Стоп-кадр» =====

class FreezeFrameScreen extends ConsumerStatefulWidget {
  const FreezeFrameScreen({super.key});
  @override
  ConsumerState<FreezeFrameScreen> createState() => _FreezeFrameScreenState();
}

class _FreezeFrameScreenState extends ConsumerState<FreezeFrameScreen> {
  List<FreezeEntry>? _entries;
  bool _loading = false;
  String? _error;

  Future<void> _read() async {
    final svc = ref.read(connectionProvider.notifier).service;
    if (svc == null) return;
    setState(() {
      _loading = true;
      _error = null;
    });
    try {
      final e = await svc.readFreezeFrame();
      setState(() => _entries = e);
    } catch (e) {
      setState(() => _error = "$e");
    } finally {
      setState(() => _loading = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    final conn = ref.watch(connectionProvider);
    return Scaffold(
      backgroundColor: const Color(0xFF0C100F),
      appBar: AppBar(
        backgroundColor: const Color(0xFF171D1C),
        title: const Text("Стоп-кадр"),
        actions: [_StatusChip(conn.link)],
      ),
      body: !conn.isConnected
          ? const _NotConnected()
          : Padding(
              padding: const EdgeInsets.all(16),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: [
                  const Text(
                      "Параметры, зафиксированные ЭБУ в момент возникновения ошибки.",
                      style: TextStyle(color: Colors.white54, fontSize: 13)),
                  const SizedBox(height: 12),
                  FilledButton.icon(
                    icon: const Icon(Icons.download),
                    label: const Text("Прочитать стоп-кадр"),
                    onPressed: _loading ? null : _read,
                  ),
                  if (_loading) ...[
                    const SizedBox(height: 12),
                    const LinearProgressIndicator(),
                  ],
                  if (_error != null) ...[
                    const SizedBox(height: 12),
                    Text(_error!, style: const TextStyle(color: Colors.redAccent)),
                  ],
                  const SizedBox(height: 12),
                  Expanded(
                    child: _entries == null
                        ? const Center(
                            child: Text("Нажмите «Прочитать стоп-кадр»",
                                style: TextStyle(color: Colors.white38)))
                        : _entries!.isEmpty
                            ? const Center(
                                child: Text("Стоп-кадр пуст",
                                    style: TextStyle(color: Colors.white38)))
                            : ListView(
                                children: _entries!
                                    .map((e) => ListTile(
                                          dense: true,
                                          title: Text(e.label,
                                              style: const TextStyle(
                                                  color: Colors.white70)),
                                          trailing: Text(
                                              "${e.value.toStringAsFixed(1)} ${e.unit}",
                                              style: const TextStyle(
                                                  color: Colors.cyanAccent,
                                                  fontSize: 16)),
                                        ))
                                    .toList(),
                              ),
                  ),
                ],
              ),
            ),
    );
  }
}


// ===== экран «Идентификаторы ЭБУ» =====

class EcuIdScreen extends ConsumerStatefulWidget {
  const EcuIdScreen({super.key});
  @override
  ConsumerState<EcuIdScreen> createState() => _EcuIdScreenState();
}

class _EcuIdScreenState extends ConsumerState<EcuIdScreen> {
  String? _vin;
  String? _ecu;
  bool _loading = false;
  String? _error;

  Future<void> _read() async {
    final svc = ref.read(connectionProvider.notifier).service;
    if (svc == null) return;
    setState(() {
      _loading = true;
      _error = null;
    });
    try {
      final vin = await svc.readVin();
      final ecu = await svc.readEcuName();
      setState(() {
        _vin = vin;
        _ecu = ecu;
      });
    } catch (e) {
      setState(() => _error = "$e");
    } finally {
      setState(() => _loading = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    final conn = ref.watch(connectionProvider);
    return Scaffold(
      backgroundColor: const Color(0xFF0C100F),
      appBar: AppBar(
        backgroundColor: const Color(0xFF171D1C),
        title: const Text("Идентификаторы ЭБУ"),
        actions: [_StatusChip(conn.link)],
      ),
      body: !conn.isConnected
          ? const _NotConnected()
          : Padding(
              padding: const EdgeInsets.all(16),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: [
                  FilledButton.icon(
                    icon: const Icon(Icons.download),
                    label: const Text("Считать идентификаторы"),
                    onPressed: _loading ? null : _read,
                  ),
                  if (_loading) ...[
                    const SizedBox(height: 12),
                    const LinearProgressIndicator(),
                  ],
                  if (_error != null) ...[
                    const SizedBox(height: 12),
                    Text(_error!, style: const TextStyle(color: Colors.redAccent)),
                  ],
                  const SizedBox(height: 16),
                  _idCard("VIN", _vin),
                  const SizedBox(height: 12),
                  _idCard("Имя/калибровка ЭБУ", _ecu),
                ],
              ),
            ),
    );
  }

  Widget _idCard(String label, String? value) {
    return Card(
      color: const Color(0xFF171D1C),
      child: ListTile(
        title: Text(label, style: const TextStyle(color: Colors.white54, fontSize: 13)),
        subtitle: SelectableText(
          value ?? "—",
          style: const TextStyle(
              color: Colors.cyanAccent, fontSize: 18, fontWeight: FontWeight.w600),
        ),
      ),
    );
  }
}


// ===== экран «Тесты на выбросы» (готовность мониторов) =====

class EmissionsScreen extends ConsumerStatefulWidget {
  const EmissionsScreen({super.key});
  @override
  ConsumerState<EmissionsScreen> createState() => _EmissionsScreenState();
}

class _EmissionsScreenState extends ConsumerState<EmissionsScreen> {
  Readiness? _data;
  bool _loading = false;
  String? _error;

  Future<void> _read() async {
    final svc = ref.read(connectionProvider.notifier).service;
    if (svc == null) return;
    setState(() {
      _loading = true;
      _error = null;
    });
    try {
      final r = await svc.readReadiness();
      setState(() => _data = r);
    } catch (e) {
      setState(() => _error = "$e");
    } finally {
      setState(() => _loading = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    final conn = ref.watch(connectionProvider);
    final d = _data;
    return Scaffold(
      backgroundColor: const Color(0xFF0C100F),
      appBar: AppBar(
        backgroundColor: const Color(0xFF171D1C),
        title: const Text("Тесты на выбросы"),
        actions: [_StatusChip(conn.link)],
      ),
      body: !conn.isConnected
          ? const _NotConnected()
          : Padding(
              padding: const EdgeInsets.all(16),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: [
                  FilledButton.icon(
                    icon: const Icon(Icons.download),
                    label: const Text("Проверить готовность"),
                    onPressed: _loading ? null : _read,
                  ),
                  if (_loading) ...[
                    const SizedBox(height: 12),
                    const LinearProgressIndicator(),
                  ],
                  if (_error != null) ...[
                    const SizedBox(height: 12),
                    Text(_error!, style: const TextStyle(color: Colors.redAccent)),
                  ],
                  if (d != null) ...[
                    const SizedBox(height: 16),
                    Row(
                      children: [
                        Icon(d.milOn ? Icons.warning : Icons.check_circle,
                            color: d.milOn ? Colors.orangeAccent : Colors.greenAccent),
                        const SizedBox(width: 8),
                        Text(
                          d.milOn
                              ? "Check Engine горит • ошибок: ${d.dtcCount}"
                              : "Check Engine не горит • ошибок: ${d.dtcCount}",
                          style: const TextStyle(color: Colors.white),
                        ),
                      ],
                    ),
                    const SizedBox(height: 12),
                    Expanded(
                      child: ListView(
                        children: d.monitors
                            .where((m) => m.supported)
                            .map((m) => ListTile(
                                  dense: true,
                                  leading: Icon(
                                    m.complete ? Icons.check_circle : Icons.hourglass_bottom,
                                    color: m.complete ? Colors.greenAccent : Colors.amber,
                                  ),
                                  title: Text(m.name,
                                      style: const TextStyle(color: Colors.white70)),
                                  trailing: Text(
                                    m.complete ? "Готов" : "Не готов",
                                    style: TextStyle(
                                        color: m.complete
                                            ? Colors.greenAccent
                                            : Colors.amber),
                                  ),
                                ))
                            .toList(),
                      ),
                    ),
                  ],
                ],
              ),
            ),
    );
  }
}


// ===== экран «Статистика» =====

class StatisticsScreen extends ConsumerWidget {
  const StatisticsScreen({super.key});

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final conn = ref.watch(connectionProvider);
    // подписываемся на телеметрию, чтобы экран перерисовывался по мере опроса
    ref.watch(telemetryProvider);
    final svc = ref.read(connectionProvider.notifier).service;
    final stats = svc?.statistics ?? const <String, MinMax>{};

    return Scaffold(
      backgroundColor: const Color(0xFF0C100F),
      appBar: AppBar(
        backgroundColor: const Color(0xFF171D1C),
        title: const Text("Статистика"),
        actions: [
          if (svc != null)
            IconButton(
              icon: const Icon(Icons.refresh),
              tooltip: "Сбросить",
              onPressed: () => svc.resetStatistics(),
            ),
          _StatusChip(conn.link),
        ],
      ),
      body: !conn.isConnected
          ? const _NotConnected()
          : stats.isEmpty
              ? const Center(
                  child: Text("Накапливаются данные…",
                      style: TextStyle(color: Colors.white38)))
              : ListView(
                  children: [
                    const Padding(
                      padding: EdgeInsets.fromLTRB(16, 12, 16, 4),
                      child: Row(
                        children: [
                          Expanded(flex: 3, child: Text("Параметр", style: TextStyle(color: Colors.white38))),
                          Expanded(child: Text("Мин", style: TextStyle(color: Colors.white38))),
                          Expanded(child: Text("Тек", style: TextStyle(color: Colors.white38))),
                          Expanded(child: Text("Макс", style: TextStyle(color: Colors.white38))),
                        ],
                      ),
                    ),
                    ...stats.entries.map((e) {
                      final s = e.value;
                      return Padding(
                        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                        child: Row(
                          children: [
                            Expanded(
                                flex: 3,
                                child: Text(e.key,
                                    style: const TextStyle(color: Colors.white70))),
                            Expanded(
                                child: Text(s.min.toStringAsFixed(0),
                                    style: const TextStyle(color: Colors.lightBlueAccent))),
                            Expanded(
                                child: Text(s.last.toStringAsFixed(0),
                                    style: const TextStyle(color: Colors.white))),
                            Expanded(
                                child: Text(s.max.toStringAsFixed(0),
                                    style: const TextStyle(color: Colors.orangeAccent))),
                          ],
                        ),
                      );
                    }),
                  ],
                ),
    );
  }
}


// ===== экран «Запись данных» =====

class DataLoggingScreen extends ConsumerStatefulWidget {
  const DataLoggingScreen({super.key});
  @override
  ConsumerState<DataLoggingScreen> createState() => _DataLoggingScreenState();
}

class _DataLoggingScreenState extends ConsumerState<DataLoggingScreen> {
  StreamSubscription<Telemetry>? _sub;
  final List<Telemetry> _rows = [];
  bool _recording = false;

  void _toggle() {
    final svc = ref.read(connectionProvider.notifier).service;
    if (svc == null) return;
    if (_recording) {
      _sub?.cancel();
      setState(() => _recording = false);
    } else {
      _sub = svc.telemetry.listen((t) {
        if (mounted) setState(() => _rows.add(t));
      });
      setState(() => _recording = true);
    }
  }

  void _clear() {
    setState(() => _rows.clear());
  }

  String _csv() {
    final b = StringBuffer();
    b.writeln("rpm,speed,coolant,load,voltage,intake,fuel,maf,map,timing,ambient,fuelRate");
    for (final t in _rows) {
      b.writeln([
        t.rpm, t.speed, t.coolant, t.load, t.voltage, t.intakeTemp,
        t.fuelLevel, t.maf, t.map, t.timing, t.ambient, t.fuelRate
      ].map((v) => v == null ? "" : v.toString()).join(","));
    }
    return b.toString();
  }

  /// Сохраняет лог в файл во временной папке и открывает системный «Поделиться».
  /// Если шаринг недоступен (например, на десктопе) — показывает CSV в диалоге.
  Future<void> _export() async {
    final csv = _csv();
    try {
      final dir = await getTemporaryDirectory();
      final ts = DateTime.now().millisecondsSinceEpoch;
      final file = File("${dir.path}/obd_log_$ts.csv");
      await file.writeAsString(csv);
      await Share.shareXFiles([XFile(file.path)], text: "Лог OBD ($ts)");
    } catch (_) {
      if (mounted) _showCsv(csv);
    }
  }

  void _showCsv(String csv) {
    showDialog(
      context: context,
      builder: (_) => AlertDialog(
        backgroundColor: const Color(0xFF171D1C),
        title: Text("CSV • строк: ${_rows.length}",
            style: const TextStyle(color: Colors.white)),
        content: SizedBox(
          width: double.maxFinite,
          child: SingleChildScrollView(
            child: SelectableText(csv,
                style: const TextStyle(color: Colors.white70, fontFamily: "monospace", fontSize: 12)),
          ),
        ),
        actions: [
          TextButton(
              onPressed: () => Navigator.pop(context), child: const Text("Закрыть")),
        ],
      ),
    );
  }

  @override
  void dispose() {
    _sub?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final conn = ref.watch(connectionProvider);
    return Scaffold(
      backgroundColor: const Color(0xFF0C100F),
      appBar: AppBar(
        backgroundColor: const Color(0xFF171D1C),
        title: const Text("Запись данных"),
        actions: [_StatusChip(conn.link)],
      ),
      body: !conn.isConnected
          ? const _NotConnected()
          : Padding(
              padding: const EdgeInsets.all(16),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: [
                  Row(
                    children: [
                      Expanded(
                        child: FilledButton.icon(
                          style: FilledButton.styleFrom(
                            backgroundColor:
                                _recording ? Colors.red.shade700 : Colors.green.shade600,
                          ),
                          icon: Icon(_recording ? Icons.stop : Icons.fiber_manual_record),
                          label: Text(_recording ? "Стоп" : "Запись"),
                          onPressed: _toggle,
                        ),
                      ),
                      const SizedBox(width: 12),
                      OutlinedButton(
                        onPressed: _rows.isEmpty ? null : _export,
                        child: const Text("Экспорт"),
                      ),
                      const SizedBox(width: 8),
                      IconButton(
                        icon: const Icon(Icons.delete_outline, color: Colors.white54),
                        onPressed: _rows.isEmpty ? null : _clear,
                      ),
                    ],
                  ),
                  const SizedBox(height: 8),
                  Text("Записей: ${_rows.length}",
                      style: const TextStyle(color: Colors.white54)),
                  const SizedBox(height: 8),
                  Expanded(
                    child: _rows.isEmpty
                        ? const Center(
                            child: Text("Нет записей",
                                style: TextStyle(color: Colors.white38)))
                        : ListView.builder(
                            reverse: true,
                            itemCount: _rows.length,
                            itemBuilder: (_, i) {
                              final t = _rows[_rows.length - 1 - i];
                              return ListTile(
                                dense: true,
                                title: Text(
                                    "об ${t.rpm ?? '—'}  •  ${t.speed ?? '—'} км/ч  •  ОЖ ${t.coolant ?? '—'}°C",
                                    style: const TextStyle(
                                        color: Colors.white70, fontSize: 13)),
                              );
                            },
                          ),
                  ),
                ],
              ),
            ),
    );
  }
}


// ===== экран «Замер разгона» =====

class AccelerationScreen extends ConsumerStatefulWidget {
  const AccelerationScreen({super.key});
  @override
  ConsumerState<AccelerationScreen> createState() => _AccelerationScreenState();
}

class _AccelerationScreenState extends ConsumerState<AccelerationScreen> {
  StreamSubscription<Telemetry>? _sub;
  final _watch = Stopwatch();
  bool _armed = false;
  int _speed = 0;
  Duration? _t60;
  Duration? _t100;

  @override
  void initState() {
    super.initState();
    final svc = ref.read(connectionProvider.notifier).service;
    _sub = svc?.telemetry.listen(_onTele);
  }

  void _onTele(Telemetry t) {
    final v = t.speed;
    if (v == null) return;
    setState(() => _speed = v);

    // Старт замера: машина стояла и тронулась.
    if (_armed && !_watch.isRunning && v > 0) {
      _watch
        ..reset()
        ..start();
      _t60 = null;
      _t100 = null;
    }
    if (_watch.isRunning) {
      if (_t60 == null && v >= 60) _t60 = _watch.elapsed;
      if (_t100 == null && v >= 100) {
        _t100 = _watch.elapsed;
        _watch.stop();
        _armed = false;
      }
    }
  }

  void _arm() {
    setState(() {
      _armed = true;
      _t60 = null;
      _t100 = null;
      _watch.reset();
    });
  }

  String _fmt(Duration? d) =>
      d == null ? "—" : "${(d.inMilliseconds / 1000).toStringAsFixed(2)} с";

  @override
  void dispose() {
    _sub?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final conn = ref.watch(connectionProvider);
    return Scaffold(
      backgroundColor: const Color(0xFF0C100F),
      appBar: AppBar(
        backgroundColor: const Color(0xFF171D1C),
        title: const Text("Замер разгона"),
        actions: [_StatusChip(conn.link)],
      ),
      body: !conn.isConnected
          ? const _NotConnected()
          : Padding(
              padding: const EdgeInsets.all(24),
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: [
                  Text("$_speed",
                      textAlign: TextAlign.center,
                      style: const TextStyle(
                          color: Colors.cyanAccent,
                          fontSize: 72,
                          fontWeight: FontWeight.bold)),
                  const Text("км/ч",
                      textAlign: TextAlign.center,
                      style: TextStyle(color: Colors.white54)),
                  const SizedBox(height: 32),
                  _resultRow("0–60 км/ч", _fmt(_t60)),
                  const Divider(color: Colors.white10),
                  _resultRow("0–100 км/ч", _fmt(_t100)),
                  const SizedBox(height: 32),
                  FilledButton.icon(
                    icon: const Icon(Icons.flag),
                    label: Text(_armed ? "Ожидание старта…" : "Подготовить замер"),
                    onPressed: _armed ? null : _arm,
                  ),
                  const SizedBox(height: 8),
                  const Text(
                      "Остановитесь (0 км/ч), нажмите «Подготовить замер» и резко стартуйте.",
                      textAlign: TextAlign.center,
                      style: TextStyle(color: Colors.white38, fontSize: 12)),
                ],
              ),
            ),
    );
  }

  Widget _resultRow(String label, String value) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 8),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(label, style: const TextStyle(color: Colors.white70, fontSize: 16)),
          Text(value,
              style: const TextStyle(
                  color: Colors.greenAccent, fontSize: 20, fontWeight: FontWeight.bold)),
        ],
      ),
    );
  }
}


// ===== экран «Мои автомобили» =====

class MyCarsScreen extends ConsumerWidget {
  const MyCarsScreen({super.key});

  Future<void> _add(BuildContext context, WidgetRef ref) async {
    final nameCtrl = TextEditingController();
    final vinCtrl = TextEditingController();
    final ok = await showDialog<bool>(
      context: context,
      builder: (_) => AlertDialog(
        backgroundColor: const Color(0xFF171D1C),
        title: const Text("Новый автомобиль", style: TextStyle(color: Colors.white)),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            TextField(
              controller: nameCtrl,
              style: const TextStyle(color: Colors.white),
              decoration: const InputDecoration(labelText: "Название"),
            ),
            TextField(
              controller: vinCtrl,
              style: const TextStyle(color: Colors.white),
              decoration: const InputDecoration(labelText: "VIN (необязательно)"),
            ),
          ],
        ),
        actions: [
          TextButton(
              onPressed: () => Navigator.pop(context, false),
              child: const Text("Отмена")),
          FilledButton(
              onPressed: () => Navigator.pop(context, true),
              child: const Text("Добавить")),
        ],
      ),
    );
    if (ok == true && nameCtrl.text.trim().isNotEmpty) {
      ref.read(vehiclesProvider.notifier).add(
            Vehicle(nameCtrl.text.trim(), vinCtrl.text.trim()),
          );
    }
  }

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final cars = ref.watch(vehiclesProvider);
    return Scaffold(
      backgroundColor: const Color(0xFF0C100F),
      appBar: AppBar(
        backgroundColor: const Color(0xFF171D1C),
        title: const Text("Мои автомобили"),
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: () => _add(context, ref),
        child: const Icon(Icons.add),
      ),
      body: ListView.builder(
        itemCount: cars.length,
        itemBuilder: (_, i) {
          final c = cars[i];
          return Card(
            color: const Color(0xFF171D1C),
            margin: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
            child: ListTile(
              leading: const Icon(Icons.directions_car, color: Colors.cyanAccent),
              title: Text(c.name, style: const TextStyle(color: Colors.white)),
              subtitle: Text(c.vin.isEmpty ? "VIN не указан" : c.vin,
                  style: const TextStyle(color: Colors.white54)),
              trailing: IconButton(
                icon: const Icon(Icons.delete_outline, color: Colors.white38),
                onPressed: () => ref.read(vehiclesProvider.notifier).removeAt(i),
              ),
            ),
          );
        },
      ),
    );
  }
}


// ===== экран «Настройки» =====

class SettingsScreen extends ConsumerWidget {
  const SettingsScreen({super.key});

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final kind = ref.watch(transportKindProvider);
    final host = ref.watch(wifiHostProvider);
    final port = ref.watch(wifiPortProvider);
    final interval = ref.watch(pollIntervalMsProvider);

    return Scaffold(
      backgroundColor: const Color(0xFF0C100F),
      appBar: AppBar(
        backgroundColor: const Color(0xFF171D1C),
        title: const Text("Настройки"),
      ),
      body: ListView(
        padding: const EdgeInsets.all(16),
        children: [
          const Text("Тип адаптера",
              style: TextStyle(color: Colors.white54, fontSize: 13)),
          const SizedBox(height: 8),
          SegmentedButton<TransportKind>(
            segments: const [
              ButtonSegment(value: TransportKind.ble, label: Text("BLE")),
              ButtonSegment(value: TransportKind.wifi, label: Text("Wi-Fi")),
            ],
            selected: {kind},
            onSelectionChanged: (s) {
              final prefs = ref.read(prefsProvider);
              ref.read(transportKindProvider.notifier).state = s.first;
              prefs.setInt("transportKind", s.first.index);
            },
          ),
          const SizedBox(height: 24),
          if (kind == TransportKind.wifi) ...[
            const Text("Адрес Wi-Fi адаптера",
                style: TextStyle(color: Colors.white54, fontSize: 13)),
            const SizedBox(height: 8),
            TextFormField(
              initialValue: host,
              style: const TextStyle(color: Colors.white),
              decoration: const InputDecoration(labelText: "Хост (IP)"),
              onChanged: (v) {
                ref.read(wifiHostProvider.notifier).state = v.trim();
                ref.read(prefsProvider).setString("wifiHost", v.trim());
              },
            ),
            TextFormField(
              initialValue: port.toString(),
              style: const TextStyle(color: Colors.white),
              keyboardType: TextInputType.number,
              decoration: const InputDecoration(labelText: "Порт"),
              onChanged: (v) {
                final p = int.tryParse(v.trim());
                if (p != null) {
                  ref.read(wifiPortProvider.notifier).state = p;
                  ref.read(prefsProvider).setInt("wifiPort", p);
                }
              },
            ),
            const SizedBox(height: 24),
          ],
          Text("Частота опроса: $interval мс",
              style: const TextStyle(color: Colors.white54, fontSize: 13)),
          Slider(
            value: interval.toDouble().clamp(50, 1000).toDouble(),
            min: 50,
            max: 1000,
            divisions: 19,
            label: "$interval мс",
            onChanged: (v) {
              ref.read(pollIntervalMsProvider.notifier).state = v.round();
              ref.read(prefsProvider).setInt("pollIntervalMs", v.round());
            },
          ),
          const SizedBox(height: 8),
          const Text(
              "Меньше значение — чаще обновление, но выше нагрузка на адаптер. "
              "Изменения применяются при следующем подключении.",
              style: TextStyle(color: Colors.white38, fontSize: 12)),
        ],
      ),
    );
  }
}


// ===== из main.dart =====

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  final prefs = await SharedPreferences.getInstance();
  runApp(
    ProviderScope(
      overrides: [prefsProvider.overrideWithValue(prefs)],
      child: const ObdApp(),
    ),
  );
}

class ObdApp extends StatelessWidget {
  const ObdApp({super.key});

  @override
  Widget build(BuildContext context) {
    final scheme = ColorScheme.fromSeed(
      seedColor: AppColors.accent,
      brightness: Brightness.dark,
    ).copyWith(
      surface: AppColors.surface,
      secondary: AppColors.accent2,
    );
    return MaterialApp(
      title: "Revoscan",
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        useMaterial3: true,
        brightness: Brightness.dark,
        scaffoldBackgroundColor: AppColors.bg,
        colorScheme: scheme,
        fontFamily: 'Roboto',
        snackBarTheme: const SnackBarThemeData(
          backgroundColor: AppColors.surface2,
          contentTextStyle: TextStyle(color: AppColors.text),
        ),
      ),
      home: const RootShell(),
    );
  }
}
