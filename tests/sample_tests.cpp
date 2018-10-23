#include <catch.hpp>

struct Foo
{
    bool is_bar() const
    {
        return true;
    }
};

// Check out https://github.com/catchorg/Catch2 for more information about how to use Catch2
// You can run the test binary with the command-line flag --help to see tweak what is tested
// and what is displayed
TEST_CASE( "Foo is always Bar", "[Foobar]" )
{
    Foo foo;

    REQUIRE(foo.is_bar());
    CHECK(foo.is_bar());
}
