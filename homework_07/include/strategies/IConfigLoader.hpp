class IConfigLoader {
public:
    virtual void loadConfig(const char* filename) = 0;
    virtual ~IConfigLoader() = default;
};